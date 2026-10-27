"""Rejouer la comparaison GEO V3 sur PC, sans modifier les sources actives."""
from pathlib import Path
import argparse
import json
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vues", type=int, default=20)
    args = parser.parse_args()
    if args.vues < 1:
        parser.error("Le nombre de vues doit etre positif.")
    directory = Path(__file__).resolve().parent
    root = directory.parents[1]
    compiler = shutil.which("gcc")
    if not compiler:
        raise SystemExit("GCC est necessaire pour compiler le banc C.")
    with tempfile.TemporaryDirectory(prefix="geov3_comparaison_") as scratch:
        scratch = Path(scratch)
        modeles = scratch / "modeles"
        modeles.mkdir()
        # Les fichiers GS peuvent employer CR seul : normaliser seulement les copies.
        for source in (root / "models").iterdir():
            if source.is_file():
                (modeles / source.name).write_text(
                    source.read_text(encoding="utf-8-sig", errors="replace"),
                    encoding="utf-8", newline="\n")
        executable = scratch / "comparaison.exe"
        subprocess.run([compiler, "-std=c99", "-O2", "-fwrapv",
                        str(directory / "comparaison_geov3.c"), "-lm", "-o", str(executable)],
                       check=True)
        log = directory / "nouvelle_execution.log"
        with log.open("w", encoding="utf-8") as output:
            subprocess.run([str(executable), str(args.vues), str(modeles)],
                           cwd=root, stdout=output, stderr=subprocess.STDOUT, check=True)
    rows = [json.loads(line[7:]) for line in log.read_text(encoding="utf-8").splitlines()
            if line.startswith("RESULT ")]
    (directory / "nouveaux_resultats.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    complete = [row for row in rows
                if not row["old_qd_fallbacks"] and not row["new_qd_fallbacks"]]
    print(f"{len(rows)} configurations, {len(complete)} sans repli QuickDraw indisponible.")
    print(f"Visibilite differente : {sum(row['pixels_changed'] > 0 for row in complete)} configurations completes.")
    print("Les resultats de l'audit initial sont conserves ; voir nouveaux_resultats.json.")


if __name__ == "__main__":
    main()
