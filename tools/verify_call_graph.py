"""Vérifier ou synchroniser les deux graphes HTML depuis les sources incluses.

Usage : python tools/verify_call_graph.py [--update] [--preprocess]
Le prétraitement contrôle les seize combinaisons des options assembleur.
Ce script ne compile pas et ne modifie jamais les sources du programme.
"""

import argparse
import itertools
import re
import sys
from html import escape, unescape
from html.parser import HTMLParser
from pathlib import Path

# Un contrôle documentaire ne doit pas ajouter de cache Python au dépôt.
sys.dont_write_bytecode = True
import call_graph_source as source

ROOT = source.ROOT
GRAPH = source.G
FUNCTIONS = source.RAW
PAGES = ("arbre_appels.html", "arbre_appels_fold.html")
GROUPS = {
    "Appels internes directs :": "direct",
    "Appels internes indirects :": "indirect",
    "Appelants internes directs :": "callers",
    "Appelants internes indirects :": "indirect_callers",
}
CALLERS = {kind: {name: set() for name in FUNCTIONS} for kind in ("direct", "indirect")}
for kind in CALLERS:
    for caller, targets in GRAPH[kind].items():
        for target in targets:
            CALLERS[kind][target].add(caller)


def ordered(names):
    """Trier les identifiants de manière stable et indépendante de la casse."""
    return sorted(names, key=str.lower)


def roles():
    """Conserver les descriptions existantes : cet audit porte sur les appels."""
    text = (ROOT / PAGES[0]).read_text(encoding="utf-8-sig")
    return {
        match[1]: unescape(re.sub("<[^>]+>", "", match[2]))
        for match in re.finditer(
            r'<details class="function" id="fn-([^"]+)".*?<h4>Rôle :</h4>\s*<p>(.*?)</p>',
            text, re.S,
        )
    }


ROLES = roles()
assert set(ROLES) == set(FUNCTIONS), "Définitions et fiches documentaires différentes"


def link(name, attrs=""):
    """Créer un lien de fonction dont le rôle reste accessible au survol."""
    return ('<a href="#fn-' + name + '" title="' + escape(ROLES[name], quote=True)
            + '"' + attrs + '><code>' + name + '</code></a>')


def links(names):
    """Formater une liste de relations, y compris une liste vide explicite."""
    return ", ".join(link(name) for name in ordered(names)) or "Aucun dans le périmètre."


def full_tree():
    """Développer chaque fonction une fois, puis référencer les cibles partagées."""
    expanded = set()

    def node(name, caller=None, kind=None, path=()):
        attrs = ""
        if caller is not None:
            attrs = (f' data-caller="{caller}" data-callee="{name}" data-kind="{kind}"')
        title = link(name, attrs)
        if kind == "indirect":
            title += ' <span class="muted">(callback)</span>'
        if name in path:
            return title + ' <span class="muted">(cycle du graphe statique)</span>'
        if name in expanded:
            return title + ' <span class="muted">(déjà détaillée)</span>'
        expanded.add(name)
        children = [(target, k) for k in ("direct", "indirect")
                    for target in ordered(GRAPH[k][name])]
        if not children:
            return title
        result = (f'<details class="branch" data-function="{name}"'
                  + (' open' if caller is None else '') + '><summary>' + title
                  + (' — point d’entrée' if caller is None else '') + '</summary><ul>')
        for target, child_kind in children:
            result += '<li>' + node(target, name, child_kind, path + (name,)) + '</li>'
        return result + '</ul></details>'

    result = node("main")
    assert expanded == source.reachable(GRAPH)
    return result


class PageAudit(HTMLParser):
    """Contrôler aussi l'imbrication effective, et pas seulement les liens HTML."""

    VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link",
            "meta", "param", "source", "track", "wbr"}

    def __init__(self):
        super().__init__()
        self.tags = []
        self.branches = []
        self.in_summary = 0
        self.function = None
        self.group = None
        self.heading = None
        self.cards = {}
        self.tree = []
        self.ids = set()
        self.anchors = set()
        self.local_links = []
        self.errors = []

    def handle_starttag(self, tag, attributes):
        attrs = dict(attributes)
        if tag not in self.VOID:
            self.tags.append((tag, attrs))
        if "id" in attrs:
            if attrs["id"] in self.ids:
                self.errors.append("Identifiant dupliqué : " + attrs["id"])
            self.ids.add(attrs["id"])
        if tag == "details" and attrs.get("class") == "branch":
            self.branches.append(attrs.get("data-function"))
        if tag == "summary":
            self.in_summary += 1
        if tag == "details" and attrs.get("class") == "function":
            self.function = attrs["id"][3:]
            self.cards[self.function] = {group: set() for group in GROUPS.values()}
        if tag == "h4" and self.function:
            self.heading = ""
        if tag == "a" and attrs.get("href", "").startswith("#"):
            self.anchors.add(attrs["href"][1:])
            if attrs["href"].startswith("#fn-") and self.function and self.group:
                self.cards[self.function][self.group].add(attrs["href"][4:])
        if tag == 'a' and attrs.get('href') and not re.match(r'^[a-zA-Z]+:|^#', attrs['href']):
            self.local_links.append(attrs['href'])
        if tag == "a" and "data-caller" in attrs:
            caller, callee, kind = (attrs[k] for k in ("data-caller", "data-callee", "data-kind"))
            parent = (self.branches[-2] if self.in_summary and len(self.branches) >= 2
                      else self.branches[-1] if self.branches else None)
            if parent != caller:
                self.errors.append(f"Imbrication fausse : {callee} sous {parent}, attendu {caller}")
            if attrs.get("href") != "#fn-" + callee:
                self.errors.append("Cible de lien incohérente : " + callee)
            self.tree.append((caller, callee, kind))

    def handle_data(self, text):
        if self.heading is not None:
            self.heading += text

    def handle_endtag(self, tag):
        if tag in self.VOID:
            return
        if not self.tags or self.tags[-1][0] != tag:
            self.errors.append("Balise mal imbriquée : " + tag)
            return
        _, attrs = self.tags.pop()
        if tag == "summary":
            self.in_summary -= 1
        if tag == "h4" and self.function:
            self.group = GROUPS.get(self.heading)
            self.heading = None
        if tag == "details" and attrs.get("class") == "function":
            self.function = self.group = None
        if tag == "details" and attrs.get("class") == "branch":
            self.branches.pop()

    def verify(self):
        """Comparer chaque fiche et chaque branche aux matrices issues du code."""
        assert not self.errors, self.errors[:10]
        assert not self.tags and not self.branches, "Balises non fermées"
        assert not self.anchors - self.ids, self.anchors - self.ids
        for href in self.local_links:
            filename, _, fragment = href.partition('#')
            target = ROOT / filename
            assert target.is_file(), ('Fichier lié absent', href)
            if fragment:
                ids = set(re.findall(r'\bid="([^"]+)"', target.read_text(encoding='utf-8-sig')))
                assert fragment in ids, ('Ancre externe absente', href)
        assert set(self.cards) == set(FUNCTIONS), "Fiches absentes ou périmées"
        for name, card in self.cards.items():
            expected = {
                "direct": GRAPH["direct"][name], "indirect": GRAPH["indirect"][name],
                "callers": CALLERS["direct"][name],
                "indirect_callers": CALLERS["indirect"][name],
            }
            assert card == expected, (name, card, expected)
        reachable = source.reachable(GRAPH)
        expected = {(caller, callee, kind) for kind in ("direct", "indirect")
                    for caller in reachable for callee in GRAPH[kind][caller]}
        assert set(self.tree) == expected, ("Relations de l’arbre", set(self.tree) ^ expected)
        assert len(self.tree) == len(expected), "Relations affichées plusieurs fois"


def sync_pages():
    """Synchroniser le parcours complet, les fiches et leurs relations inverses."""
    tree = full_tree()
    for filename in PAGES:
        path = ROOT / filename
        text = path.read_text(encoding="utf-8-sig")
        start = text.index("<h2>Parcours")
        end = text.index("<h2>Répertoire des fonctions", start)
        text = text[:start] + (
            '<h2>Parcours complet depuis main</h2><p class="muted">'
            'Chaque fonction est détaillée une fois ; les liens suivants renvoient à sa fiche. '
            'Les callbacks sont signalés. Un cycle du graphe statique ne prouve pas une '
            'récursion effective : les conditions d’exécution peuvent l’empêcher.</p>'
        ) + tree + '\n' + text[end:]

        def update(match):
            name = match[1]
            card = match[0]
            card = re.sub(r'(· ligne )\d+', lambda m: m[1] + str(FUNCTIONS[name]["line"]), card)
            pattern = r'<h4>Appels internes directs :</h4>.*?(?=<p><a href="[^#]|$)'
            relations = '<h4>Appels internes directs :</h4><p>' + links(GRAPH["direct"][name]) + '</p>'
            relations += '<h4>Appels internes indirects :</h4><p>' + links(GRAPH["indirect"][name]) + '</p>'
            if name == 'painter_newell_sancha_fast':
                relations += '<p class="muted">Comparateur invoqué par qsort.</p>'
            if name == 'geov3_run':
                relations += ('<p class="muted">Callback on_exchange fourni par repair_geov3 ; '
                              'painter_geoV3 passe NULL et ne déclenche pas ce callback.</p>')
            relations += '<h4>Appelants internes directs :</h4><p>' + links(CALLERS["direct"][name]) + '</p>'
            relations += '<h4>Appelants internes indirects :</h4><p>' + links(CALLERS["indirect"][name]) + '</p>'
            if GRAPH["local"][name]:
                relations += ('<h4>Sous-routines assembleur locales :</h4><p>'
                              + ', '.join('<code>' + n + '</code>' for n in ordered(GRAPH["local"][name]))
                              + ' — labels internes à cette fonction, sans fiche C distincte.</p>')
            if GRAPH["dynamic"][name]:
                relations += ('<h4>Exécution dynamique :</h4><p>JSL auto-modifiant vers les blocs '
                              'de transition générés en RAM ; ce ne sont pas des fonctions C du projet.</p>')
            if GRAPH["external"][name]:
                relations += ('<h4>Appels hors du projet :</h4><p class="muted">'
                              + ', '.join('<code>' + escape(n) + '</code>' for n in ordered(GRAPH["external"][name]))
                              + '.</p>')
            card, count = re.subn(pattern, lambda m: relations + '\n', card, count=1, flags=re.S)
            assert count == 1, name
            return card

        text = re.sub(r'<details class="function" id="fn-([^"]+)".*?</details>', update, text, flags=re.S)
        stats = ('<div class="stats">'
                 f'<div class="stat"><strong>{len(FUNCTIONS)}</strong><br>définitions de fonctions</div>'
                 f'<div class="stat"><strong>{len(source.sources)}</strong><br>fichiers actifs inclus</div>'
                 f'<div class="stat"><strong>{sum(map(len, GRAPH["direct"].values()))}</strong><br>relations directes</div>'
                 f'<div class="stat"><strong>{sum(map(len, GRAPH["indirect"].values()))}</strong><br>relations par callback</div></div>')
        text = re.sub(r'<div class="stats">.*?</div></div>', lambda m: stats, text, count=1, flags=re.S)
        note = ('<aside class="note"><strong>Portée de la vérification</strong><p>'
                'Le parcours transitif part de main et distingue appels C/assembleur directs et callbacks. '
                'Les branches conditionnelles sont réunies : le graphe décrit les chemins possibles, '
                'pas une trace d’exécution. Les macros sans appel de fonction et les dépendances par '
                'données globales ne sont pas des relations d’appel.</p><p>'
                'Le périmètre suit les inclusions locales de ObjExplorer.cc ; old/, tools/ et DEPLOY.py '
                'sont exclus. Les labels assembleur locaux et le code généré à l’exécution sont signalés séparément. '
                '<a href="AUDIT_ARBRE_APPELS.md">Méthode, résultats et contrôle reproductible</a>.</p></aside>')
        text = re.sub(r'<aside class="note">.*?</aside>', lambda m: note, text, count=1, flags=re.S)
        text = re.sub(r'<footer>.*?</footer>', lambda m: (
            '<footer>Relations d’appel vérifiées depuis les sources. '
            '<a href="AUDIT_ARBRE_APPELS.md">Audit, limites et procédure de contrôle</a>. '
            'Un graphe statique ne garantit ni l’exécution d’un chemin ni l’équivalence des algorithmes.</footer>'
        ), text, count=1, flags=re.S)
        if 'const branches = ' not in text:
            text = text.replace("const status = document.getElementById('status');",
                                "const branches = [...document.querySelectorAll('details.branch')];\nconst status = document.getElementById('status');")
            text = text.replace('for (const card of cards) if (!card.hidden) card.open = true;',
                                'for (const card of cards) if (!card.hidden) card.open = true;\n  for (const branch of branches) branch.open = true;')
            text = text.replace('for (const card of cards) card.open = false;',
                                'for (const card of cards) card.open = false;\n  for (const branch of branches) branch.open = false;')
        path.write_text(text, encoding="utf-8", newline="\n")


def check_pages():
    """Vérifier les deux variantes et imposer des relations strictement identiques."""
    snapshots = []
    for filename in PAGES:
        audit = PageAudit()
        audit.feed((ROOT / filename).read_text(encoding="utf-8-sig"))
        audit.verify()
        snapshots.append((audit.cards, set(audit.tree)))
    assert snapshots[0] == snapshots[1], "Les variantes ouverte et repliée divergent"
    assert not any(GRAPH['refs'].values()), "Références de fonctions non résolues à examiner"
    for filename in PAGES:
        text = (ROOT / filename).read_text(encoding='utf-8-sig')
        for name, body in re.findall(r'<details class="function" id="fn-([^"]+)"(.*?)</details>', text, re.S):
            signature = re.search(r'<pre><code>(.*?)</code></pre>', body, re.S)
            expected = [token for token, _ in source.lex(FUNCTIONS[name]['signature'])]
            assert signature and [token for token, _ in source.lex(unescape(signature[1]))] == expected, ('Signature', name)
            location = re.search(r'<span class="muted">([^<]+) · ligne (\d+)</span>', body)
            assert location and location[1].lower() == FUNCTIONS[name]['file'].lower(), ('Fichier', name)
            assert int(location[2]) == FUNCTIONS[name]['line'], ('Ligne', name)


def dictionary_blocks(text):
    """Isoler les contrats sans confondre leur index et leurs ancres."""
    return re.findall(r'(<a id="fn-([^"]+)"></a>.*?)(?=<a id="fn-|\Z)', text, re.S)


def sync_dictionary():
    """Synchroniser les relations et les lignes, en conservant les contrats rédigés."""
    path = ROOT / 'dictionary.md'
    text = path.read_text(encoding='utf-8-sig')
    text = text.replace('[bilan de révision](REVISION_DOCUMENTATION.md)',
                        '[audit des appels](AUDIT_ARBRE_APPELS.md)')

    def update(match):
        body, name = match[0], match[2]
        body = re.sub(r'(Définition : [^\n]+, ligne )\d+',
                      lambda m: m[1] + str(FUNCTIONS[name]['line']), body, count=1)
        mdlinks = lambda names: ', '.join(f'[{n}](arbre_appels.html#fn-{n})' for n in ordered(names)) or 'aucun dans le périmètre'
        body = re.sub(r'Appels internes directs : [^\n]*',
                      lambda m: 'Appels internes directs : ' + mdlinks(GRAPH['direct'][name]) + '.', body, count=1)
        body = re.sub(r'\nAppels internes indirects : [^\n]*\n', '\n', body)
        if GRAPH['indirect'][name]:
            body = body.rstrip() + '\n\nAppels internes indirects : ' + mdlinks(GRAPH['indirect'][name]) + '.\n\n'
        return body

    text = re.sub(r'(<a id="fn-([^"]+)"></a>.*?)(?=<a id="fn-|\Z)', update, text, flags=re.S)
    note = 'Les relations d’appel sont vérifiées depuis `main` ; voir [l’audit reproductible](AUDIT_ARBRE_APPELS.md). Les callbacks sont distingués des appels directs.\n\n'
    if note not in text:
        pos = text.index('## Repères et formats')
        text = text[:pos] + note + text[pos:]
    path.write_text(text, encoding='utf-8', newline='\n')


def check_dictionary():
    """Contrôler toutes les listes d'appels du dictionnaire contre la même source."""
    blocks = dictionary_blocks((ROOT / 'dictionary.md').read_text(encoding='utf-8-sig'))
    assert {name for _, name in blocks} == set(FUNCTIONS), 'Périmètre du dictionnaire incohérent'
    for body, name in blocks:
        for heading, kind in [('directs', 'direct'), ('indirects', 'indirect')]:
            match = re.search(r'Appels internes ' + heading + r' : ([^\n]*)', body)
            targets = set(re.findall(r'arbre_appels.html#fn-([^\)]+)', match[1])) if match else set()
            assert targets == GRAPH[kind][name], ('Dictionnaire', name, kind)


def check_preprocessing():
    """Confirmer tous les arcs directs dans les seize configurations assembleur."""
    flags = ('GEOV3_DISABLE_ASM_MUL', 'GEOV3_DISABLE_ASM_RELATION',
             'GEOV3_DISABLE_ASM_SCAN', 'FACE_DEPTH_DISABLE_ASM')
    union = set()
    results = []
    for bits in itertools.product((False, True), repeat=len(flags)):
        enabled = [flag for flag, active in zip(flags, bits) if active]
        definitions, graph = source.preprocess(enabled)
        edges = source.flatten(graph['direct'])
        assert edges <= source.flatten(GRAPH['direct']), enabled
        union |= edges
        results.append((enabled, len(definitions), len(edges), len(source.reachable(graph))))
    assert union == source.flatten(GRAPH['direct']), "Arcs non confirmés après prétraitement"
    return results


def self_test():
    """Injecter sept défauts en mémoire pour éprouver le contrôleur sans toucher aux pages."""
    text = (ROOT / PAGES[1]).read_text(encoding='utf-8-sig')
    cases = [
        text.replace('data-caller="main" data-callee="animate_model_rotation"',
                     'data-caller="check_intersect" data-callee="animate_model_rotation"', 1),
        text.replace('href="#fn-main"', 'href="#fn-fonction_absente"', 1),
        text.replace('href="README.md"', 'href="fichier_absent.md"', 1),
        text.replace('id="fn-main"', 'id="fn-animation_fit"', 1),
        text.replace('<h4>Appels internes directs :</h4><p>',
                     '<h4>Appels internes directs :</h4><p><a href="#fn-main">main</a>, ', 1),
        re.sub(r'(<details class="function" id="fn-geov3_run".*?<h4>Appels internes indirects :</h4><p>).*?(</p>)',
               lambda m: m[1] + 'Aucun.' + m[2], text, count=1, flags=re.S),
    ]
    for bad in cases:
        assert bad != text, 'Le défaut doit avoir été injecté'
        audit = PageAudit()
        audit.feed(bad)
        try:
            audit.verify()
        except AssertionError:
            pass
        else:
            raise AssertionError('Défaut documentaire non détecté')
    mini = ('<details class="branch" data-function="main"><summary>main</summary><ul><li>'
            '<details class="branch" data-function="check_intersect"><summary>check_intersect</summary><ul><li>'
            '<a href="#fn-animate_model_rotation" data-caller="main" '
            'data-callee="animate_model_rotation" data-kind="direct">animation</a>'
            '</li></ul></details></li></ul></details>')
    audit = PageAudit()
    audit.feed(mini)
    assert any('Imbrication fausse' in error for error in audit.errors)
    print('OK : sept défauts injectés et détectés, dont l’imbrication historique sous check_intersect.')


def main():
    """Appliquer les options puis échouer explicitement au premier écart documentaire."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--update', action='store_true', help='synchroniser les pages puis les vérifier')
    parser.add_argument('--preprocess', action='store_true', help='contrôler les 16 configurations avec GCC/SDK ORCA')
    parser.add_argument('--self-test', action='store_true', help='éprouver le vérificateur avec sept défauts injectés en mémoire')
    parser.add_argument('--sdk', type=Path, help='répertoire des en-têtes ORCA/C')
    args = parser.parse_args()
    if args.sdk:
        source.SDK = args.sdk
    if args.update:
        sync_pages()
        sync_dictionary()
    check_pages()
    check_dictionary()
    results = check_preprocessing() if args.preprocess else []
    if args.self_test:
        self_test()
    print(f'OK : {len(FUNCTIONS)} fonctions, {len(source.reachable(GRAPH))} accessibles depuis main, '
          f'{sum(map(len, GRAPH["direct"].values()))} relations directes, '
          f'{sum(map(len, GRAPH["indirect"].values()))} callbacks ; 2 pages cohérentes.')
    if results:
        print(f'OK : {len(results)} configurations prétraitées, tous les arcs directs confirmés.')


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
