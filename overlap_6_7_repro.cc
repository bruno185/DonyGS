/* Regression: models/a, faces 6/7, H=216 V=15 W=0, culling off.
 * Run from the project directory. A positive ray result means face 7 is closer.
 * Compile/link with ORCA/C through Golden Gate, then run with iix.
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"

static int check_fractional_pair(Model3D *model, const char *label)
{
    double x=0.0, y=0.0, area2=0.0;
    int ok=compute_intersection_centroid_fractional_tryboth(model,6,7,&x,&y,&area2);
    int relation=ok?ray_cast_at_fractional(model,6,7,x,y):0;
    printf("%s: SH=%d area=%.8f centroid=(%.8f,%.8f) ray=%d\n",
           label,ok,area2*0.5,x,y,relation);
    return ok;
}

int main(void)
{
    Model3D *model=createModel3D();
    ObserverParams params;
    RCHZoomPatch patch;
    int x=0,y=0,integer_ok,passed;
    long long area2=0;
    if(!model || loadModel3D(model,"models/a")<0)exit(1);
    s_global_proj_scale_fixed=model->auto_suggested_proj_scale;
    normalizeAutoFitDistanceTo150(model);
    params.angle_h=216;params.angle_v=15;params.angle_w=0;
    params.distance=INT_TO_FIXED(150);
    cull_back_faces=0;framePolyOnly=1;
    processModelFast(model,&params,"models/a");
    printf("Scale=%.8f distance=150, faces 6/7\n",FIXED_TO_FLOAT(s_global_proj_scale_fixed));
    {int ids[2]={6,7};int i,j;for(i=0;i<2;++i){int f=ids[i];printf("FACE %d PLANE %.12f %.12f %.12f %.12f\n",f,FIXED64_TO_FLOAT(model->faces.plane_a[f]),FIXED64_TO_FLOAT(model->faces.plane_b[f]),FIXED64_TO_FLOAT(model->faces.plane_c[f]),FIXED64_TO_FLOAT(model->faces.plane_d[f]));
    for(j=0;j<model->faces.vertex_count[f];++j){int v=model->faces.vertex_indices_buffer[model->faces.vertex_indices_ptr[f]+j]-1;printf(" v%d obs=(%.8f %.8f %.8f) screen=(%d %d)\n",v+1,FIXED_TO_FLOAT(model->vertices.xo[v]),FIXED_TO_FLOAT(model->vertices.yo[v]),FIXED_TO_FLOAT(model->vertices.zo[v]),model->vertices.x2d[v],model->vertices.y2d[v]);}}}
    printf("Geometry 6/7=%d 7/6=%d\n",geometric_face_relation(model,6,7),geometric_face_relation(model,7,6));
    printf("Overlap detection=%d\n",projected_polygons_overlap(model,6,7));
    integer_ok=compute_intersection_centroid_ordered_fixed(model,6,7,&x,&y,&area2);
    printf("Legacy integer SH: ok=%d area2=%.0f \n",integer_ok,(double)area2);
    printf("Integer centroid=(%d,%d), ray=%d\n",x,y,ray_cast_at(model,6,7,x,y));
    passed=check_fractional_pair(model,"Fractional base");
    rch_patch_pair_at_scale(model,6,7,
        FIXED_MUL_64(s_global_proj_scale_fixed,INT_TO_FIXED(10)),&patch);
    passed=check_fractional_pair(model,"Fractional x10") && passed;
    rch_unpatch_pair(model,&patch,6,7);
    {
        int forward=ray_cast_hierarchical(model,6,7);
        int reverse=ray_cast_hierarchical(model,7,6);
        printf("Hierarchy: 6/7=%d, 7/6=%d (expected +1/-1)\n",forward,reverse);
        passed=passed;
    }
    {
        int i,pos6=-1,pos7=-1;
        puts("Running complete GEO V3...");
        framePolyOnly=0;
        painter_geoV3(model,model->faces.face_count);
        for(i=0;i<model->faces.face_count;++i){if(model->faces.sorted_face_indices[i]==6)pos6=i;if(model->faces.sorted_face_indices[i]==7)pos7=i;}
        printf("GEO V3 order positions: face6=%d face7=%d (later is in front)\n",pos6,pos7);
        if(pos6<0 || pos7<0 || pos6>=pos7){puts("FAIL: face 7 must be in front");exit(1);}
        puts("PASS: face 7 is drawn after face 6");
    }
    destroyModel3D(model);
    puts("Diagnostic complete");
    exit(passed?0:1);return 0;
}
