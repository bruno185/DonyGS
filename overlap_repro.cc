/* Regression: models/a, faces 44/8, H=207 V=20 W=0, culling off.
 * Run from the project directory. A positive ray result means face 8 is closer.
 * Compile/link with ORCA/C through Golden Gate, then run with iix.
 */
#pragma memorymodel 1
#pragma optimize 1
#include "engine.c"

static int check_fractional_pair(Model3D *model, const char *label)
{
    double x=0.0, y=0.0, area2=0.0;
    int ok=compute_intersection_centroid_fractional_tryboth(model,44,8,&x,&y,&area2);
    int relation=ok?ray_cast_at_fractional(model,44,8,x,y):0;
    printf("%s: SH=%d area=%.8f centroid=(%.8f,%.8f) ray=%d\n",
           label,ok,area2*0.5,x,y,relation);
    return ok && relation==1;
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
    params.angle_h=207;params.angle_v=20;params.angle_w=0;
    params.distance=INT_TO_FIXED(150);
    cull_back_faces=0;framePolyOnly=1;
    processModelFast(model,&params,"models/a");
    printf("Scale=%.8f distance=150, faces 44/8\n",FIXED_TO_FLOAT(s_global_proj_scale_fixed));
    printf("Overlap detection=%d\n",projected_polygons_overlap(model,44,8));
    integer_ok=compute_intersection_centroid_ordered_fixed(model,44,8,&x,&y,&area2);
    printf("Legacy integer SH: ok=%d area2=%.0f (thin contour collapses)\n",integer_ok,(double)area2);
    passed=check_fractional_pair(model,"Fractional base");
    rch_patch_pair_at_scale(model,44,8,
        FIXED_MUL_64(s_global_proj_scale_fixed,INT_TO_FIXED(10)),&patch);
    passed=check_fractional_pair(model,"Fractional x10") && passed;
    rch_unpatch_pair(model,&patch,44,8);
    {
        int forward=ray_cast_hierarchical(model,44,8);
        int reverse=ray_cast_hierarchical(model,8,44);
        printf("Hierarchy: 44/8=%d, 8/44=%d (expected +1/-1)\n",forward,reverse);
        passed=passed && forward==1 && reverse==-1;
    }
    destroyModel3D(model);
    puts(passed?"PASS: face 8 is in front":"FAIL: incorrect face ordering");
    exit(passed?0:1);return 0;
}
