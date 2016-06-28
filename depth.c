#include "depth.h"
#include <math.h>
#include "bitmap.h"
#include <assert.h>
#include <intrin.h>
#include "../libutil/libutil.h"

bool DepthInit( Depth* pDep, int width, int height, char* path )
{
	int i;
	FL_FILE* pfile = NULL;
	memset( pDep, 0, sizeof(Depth) );

	srand(0);

	// default params--------------------------------------------------
	pDep->max_blobs = 10240;
	pDep->min_blob_size = 30;
	pDep->min_floor_blob_size = 20;
	pDep->odd_pt_th = 100;

	pDep->plane_thresh = 1;
	pDep->floor_y_base = 0;
	pDep->floor_x_span = 1500;
	pDep->floor_y_span = 1500;
	pDep->floor_unit   = 7;
	pDep->floor_hthresh  = 50;
	pDep->floor_wall_th  = 20;

	pDep->aim_head_r     = 150;
	pDep->aim_body_r     = 350;
	pDep->head_body_th   = 2;
	pDep->head_body_stride = 10;

	pDep->vd_learn_rate = 50;
	pDep->cam_height_adj = -1;

	pDep->active_cnt_max = 60;
	pDep->ref_support_th = 10;
	pDep->ref_people_th  = 1300;
	pDep->ref_rid_th     = 10;
	pDep->ref_rid_merge_th = 10;
	pDep->ref_merge_th_uv  = 65;
	pDep->ref_merge_th_xy  = 65;

	pDep->pt_neighbor_th = 60;
	pDep->pt_dep_dif_th  = 500;

	pDep->samples_per_pix = 10;
	pDep->dif_thresh_grey = 25;
	pDep->dif_thresh_grad = 25;
	pDep->dif_thresh_shad = 256;
	pDep->dif_thresh_hue  = 22;
	pDep->dif_thresh_dep  = 60;
	pDep->bg_cnt_thresh   = 2;
	pDep->n_subsampling   = 50;
	pDep->n_subsampling_d = 50;
	pDep->neighbor_span   = 3;

	pDep->plus_cnt        = 2;
	pDep->plus_step       = 3;
	pDep->try_step        = 24;

	pDep->cam_h_low       = 1500;
	pDep->cam_h_high      = 4000;
	pDep->fu              = (int)(1*600);
	pDep->fv              = (int)(1*570);

	pDep->track_uv_th  = 40;
	pDep->track_xy_th  = 40;
	pDep->track_height_th = 40;
	pDep->track_inner_th = 60;
	pDep->track_max_life = 10;
	pDep->track_still_th = 300;
	pDep->track_wander_th= 300;
	pDep->track_still_cnt_th = 300;
	pDep->track_run_th   = 2000;
	pDep->track_near_th  = 300;
	pDep->track_owner_th = 800;
	pDep->track_owner_cnt_th = 3;
	pDep->track_down_th  = 25;

	pDep->floor_pt_th     = 20;
	pDep->people_veri_th  = 25;
	pDep->people_area_th  = 30000;
	pDep->people_veri_cnt_th = 5;

	pDep->people_run_th   = 1;
	pDep->people_lie_th   = 65;
	pDep->people_lie_cnt_th = 1;
	pDep->people_lie_long_th = 3;
	pDep->people_stay_th  = 15;
	pDep->people_wander_dis_th = 10000;
	pDep->people_follow_th= 5;
	pDep->people_fight_th = 5;
	pDep->people_fight_det_th = 100;
	pDep->people_down_th  = 90;
	pDep->people_down_cnt_th = 3;
	pDep->object_still_th = 2;

	pDep->obj_track_th = 10;

	pDep->frame_ignore = 200;
	pDep->frame_init   = 30;

	pDep->light_up_th  = 200;
	pDep->light_down_th= 50;
	pDep->light_thresh = 150;
	pDep->blur_thresh  = 28;
	pDep->dep_valid_cnt= 60;
	pDep->cover_thresh = 500;
	pDep->floor_cnt_thresh = 7;
	pDep->cam_move_thresh  = 30;
	pDep->cover_span   = 30;

	pDep->region_in_vio_th  = 200;
	pDep->region_pre_cnt_th = 15;
	pDep->region_fbuf_th    = 150;

	pDep->ct_max_life = 7;

	// another set of initial values ( for C point )
	if( strncmp( path, "c", 9 )== 0 ){
		pDep->dif_thresh_dep = 200;
		pDep->floor_y_base = 0;
		pDep->floor_x_span = 5000;
		pDep->floor_y_span = 5000;
		pDep->floor_unit   = 18;
		pDep->ref_merge_th_uv  = 10;
		pDep->ref_merge_th_xy  = 50;
	}
	// end another set of initial values ( for C point )

	pDep->fu = pDep->fu * width  / 640;
	pDep->fv = pDep->fv * height / 480;

	memset( pDep->regions, 0, sizeof(Region)*REGION_MAX_CNT );
	memset( &pDep->valid_region, 0, sizeof(pDep->valid_region) );
	pDep->valid_region.type   = REGION_TYPE_VALID;

	for( i=0;i<256;i++ ){
		pDep->tab_r[i] = (i * 1224) >> 12;
		pDep->tab_g[i] = (i * 2404) >> 12;
		pDep->tab_b[i] = (i * 467)  >> 12;
	}

	pDep->frame_bytes = width * height * 3;
	pDep->frame_len   = width * height;
	pDep->frame_line_bytes = width * 3;

	pDep->width      = width;
	pDep->height     = height;
	pDep->pFrameDepth = (unsigned short*)malloc( pDep->frame_len * sizeof(unsigned short) );
	pDep->pFrameGrey  = (unsigned char *)malloc( pDep->frame_len );
	pDep->pFrameRGB   = (unsigned char *)malloc( pDep->frame_bytes );
	pDep->pFrameGrad  = (unsigned char *)malloc( pDep->frame_len );

	pDep->cur_status = DEPTH_STATUS_NORM;

	pDep->pObjs        = (DepObject*) malloc( sizeof(DepObject) * pDep->max_blobs );
	pDep->n_objs       = 0;
	pDep->pFinalObjs   = (DepObject*) malloc( sizeof(DepObject) * pDep->max_blobs );
	pDep->n_final_objs = 0;

	for( i=0; i<DEP_BUF_SIZE; i++ ){
		pDep->pDepBuf[i] = (unsigned short*)malloc( sizeof(unsigned short)*pDep->width*pDep->height );
		memset( pDep->pDepBuf[i], 0, sizeof(short)*pDep->width*pDep->height );
	}
	pDep->cur_dep_buf = 0;

	memset( pDep->avg_grey, 0, sizeof(pDep->avg_grey) );
	pDep->cur_avg_grey = 0;
	memset( pDep->his_DFG, 0, sizeof(pDep->his_DFG) );
	pDep->cur_dfg = 0;
	pDep->avg_depth = 0;
	pDep->dep_valid_cnt = 0;
	pDep->up_pt_cnt = 0;

	pDep->cam_fatal_cnt = 0;
	pDep->cam_ill_cnt = 0;
	pDep->cam_slash_cnt = 0;
	pDep->cam_sun_cnt = 0;
	pDep->cam_nofloor_cnt = 0;
	pDep->cam_reinit_cnt = 0;

	pDep->pImgPlaneAttr = (ImgPlaneAttr*)malloc( pDep->frame_len * sizeof(ImgPlaneAttr) );
	pDep->pPlanes = (Plane*)malloc( pDep->max_blobs * sizeof(Plane) );
	pDep->floor_attr_width = (pDep->floor_x_span + pDep->floor_unit - 1) / pDep->floor_unit;
	pDep->floor_attr_height= (pDep->floor_y_span + pDep->floor_unit - 1) / pDep->floor_unit;
	pDep->center_radius    = pDep->aim_head_r / pDep->floor_unit;
	pDep->round_radiius    = pDep->aim_body_r / pDep->floor_unit;
	pDep->cr_stride        = (pDep->head_body_stride > pDep->floor_unit) ? pDep->head_body_stride / pDep->floor_unit : 1;
	pDep->pFloorAttr = (FloorAttr*)malloc( sizeof(FloorAttr) * pDep->floor_attr_width * pDep->floor_attr_height );
	memset( pDep->pFloorAttr, 0, sizeof(FloorAttr) * pDep->floor_attr_width * pDep->floor_attr_height );
	pDep->pFloorAttrLast = (FloorAttr*)malloc( sizeof(FloorAttr) * pDep->floor_attr_width * pDep->floor_attr_height );
	memset( pDep->pFloorAttrLast, 0, sizeof(FloorAttr) * pDep->floor_attr_width * pDep->floor_attr_height );
	pDep->pFloorHeightInte = (long long*)malloc( sizeof(long long) * pDep->floor_attr_width * pDep->floor_attr_height );
	memset( pDep->pFloorHeightInte, 0, sizeof(long long) * pDep->floor_attr_width * pDep->floor_attr_height );
	pDep->pFloorFGInte = (int*)malloc( sizeof(int) * pDep->floor_attr_width * pDep->floor_attr_height );
	memset( pDep->pFloorFGInte, 0, sizeof(int) * pDep->floor_attr_width * pDep->floor_attr_height );
	pDep->pFloorPrjInte = (int*)malloc( sizeof(int) * pDep->floor_attr_width * pDep->floor_attr_height );
	memset( pDep->pFloorPrjInte, 0, sizeof(int) * pDep->floor_attr_width * pDep->floor_attr_height );
	pDep->pImgAttr = (ImgAttr*)malloc( sizeof(pDep->pImgAttr[0])*pDep->width*pDep->height );
	memset( pDep->pImgAttr, 0, sizeof(pDep->pImgAttr[0])*pDep->width*pDep->height );
	pDep->pTargets = (FloorTarget*)malloc( sizeof(FloorTarget)*pDep->max_blobs );
	memset( pDep->pTargets, 0, sizeof(FloorTarget)*pDep->max_blobs );
	pDep->floor_targets_cnt = 0;
	pDep->pPolarTargets = (PolarTarget*)malloc( sizeof(PolarTarget)*pDep->max_blobs );
	memset( pDep->pPolarTargets, 0, sizeof(PolarTarget)*pDep->max_blobs );
	pDep->pPolarQueue = (PolarQueue*)malloc( sizeof(PolarQueue)*pDep->max_blobs );
	memset( pDep->pPolarQueue, 0, sizeof(PolarQueue)*pDep->max_blobs );
	for( i=0; i<pDep->max_blobs; i++ ){
		pDep->pPolarQueue[i].queue = (PolarPoint*)malloc( sizeof(PolarPoint)*(pDep->floor_attr_width+pDep->floor_attr_height)*2 );
	}
	pDep->polar_targets_cnt = 0;
	pDep->pImgTargets = (RefineTarget*)malloc( sizeof(RefineTarget)*pDep->max_blobs );
	pDep->pRefineTargets = (RefineTarget*)malloc( sizeof(RefineTarget)*pDep->max_blobs );
	memset( pDep->pRefineTargets, 0, sizeof(RefineTarget)*pDep->max_blobs );
	for( i=0; i<pDep->max_blobs; i++ ){
		pDep->pImgTargets[i].low_pt = (char*)malloc( pDep->width );
		memset( pDep->pImgTargets[i].low_pt, 0, pDep->width );
		pDep->pImgTargets[i].rid_cnt = (int*)malloc( pDep->max_blobs*sizeof(int) );
	}
	pDep->refine_targets_cnt = 0;
	pDep->edge_up     = (short*)malloc(pDep->width*sizeof(short));
	pDep->edge_bottom = (short*)malloc(pDep->width*sizeof(short));
	pDep->edge_left   = (short*)malloc(pDep->width*sizeof(short));
	pDep->edge_right  = (short*)malloc(pDep->width*sizeof(short));

	memset( &pDep->VD, 0, sizeof(pDep->VD) );

	pDep->pModel     = (BkgModel*) malloc( sizeof(BkgModel) * pDep->frame_len * pDep->samples_per_pix );
	memset( pDep->pModel, 0, sizeof(BkgModel) * pDep->frame_len * pDep->samples_per_pix );
	pDep->pXCoeff  = (int*)malloc( pDep->frame_len * sizeof(int) );
	pDep->pYCoeff  = (int*)malloc( pDep->frame_len * sizeof(int) );
	pDep->pZCoeff  = (int*)malloc( pDep->frame_len * sizeof(int) );
	pDep->map      = (short*)malloc( (pDep->max_blobs + 1) * sizeof(short) );
	memset( pDep->map, 0, (pDep->max_blobs + 1) * sizeof(short) );
	pDep->obj_map  = (short*)malloc( (pDep->max_blobs + 1) * sizeof(short) );
	memset( pDep->obj_map, 0, (pDep->max_blobs + 1) * sizeof(short) );
	pDep->bg_targets = (BGTarget*)malloc( (pDep->max_blobs+1) * sizeof(BGTarget) );
	memset( pDep->bg_targets, 0, (pDep->max_blobs+1) * sizeof(BGTarget) );
	pDep->bg_targets_cnt = 0;
	pDep->frame_cnt      = 0;
	pDep->plus_percent   = 0;
	pDep->cam_evt        = 0;
	memset( pDep->obj_track, 0, sizeof(pDep->obj_track[0])*MAX_OBJ_TRACK );
	for( i=0; i<MAX_OBJ_TRACK; i++ ){
		pDep->obj_track[i].fg = (unsigned char*)malloc( pDep->frame_bytes );
		pDep->obj_track[i].bg = (unsigned char*)malloc( pDep->frame_bytes );
	}
	pDep->pFrameBG = (unsigned char*)malloc( pDep->frame_len );
	pDep->track_tag = (char*)malloc( pDep->max_blobs );
	memset( pDep->track_tag, 0, sizeof(char)*pDep->max_blobs );

	pDep->x = (float*)malloc(sizeof(float)*pDep->frame_len);
	pDep->y = (float*)malloc(sizeof(float)*pDep->frame_len);
	pDep->z = (float*)malloc(sizeof(float)*pDep->frame_len);

	return true;
}

void DepthUninit( Depth* pDep )
{
	int i;
	if( pDep->pFrameDepth ){
		for( i=0; i<DEP_BUF_SIZE; i++ ){
			free(pDep->pDepBuf[i]);
		}
		free( pDep->pFrameDepth );
		pDep->pFrameDepth = NULL;
		free( pDep->pFrameGrey );
		free( pDep->pFrameRGB  );
		free( pDep->pObjs ), pDep->n_objs = 0;
		//free( pDep->pRootObjs ), pDep->n_root_objs = 0;
		free( pDep->pFinalObjs ), pDep->n_final_objs = 0;
		free( pDep->pImgAttr );
		free( pDep->pPlanes );
		free( pDep->pImgPlaneAttr );
		free( pDep->pFloorAttr );
		free( pDep->pFloorAttrLast );
		free( pDep->pFloorHeightInte );
		free( pDep->pFloorFGInte );
		free( pDep->pFloorPrjInte );
		free( pDep->pTargets ), pDep->floor_targets_cnt = 0;
		free( pDep->pPolarTargets ), pDep->polar_targets_cnt = 0;
		for( i=0; i<pDep->max_blobs; i++ ){
			free( pDep->pPolarQueue[i].queue );
			pDep->pPolarQueue[i].head = pDep->pPolarQueue[i].tail = 0;
		}
		free( pDep->pPolarQueue );
		free( pDep->pRefineTargets ), pDep->refine_targets_cnt = 0;
		free( pDep->edge_up );
		free( pDep->edge_left );
		free( pDep->edge_right );
		free( pDep->edge_bottom );
		free( pDep->pXCoeff );
		free( pDep->pYCoeff );
		free( pDep->pZCoeff );
		free( pDep->pModel );
		free( pDep->map );
		free( pDep->obj_map );
		free( pDep->bg_targets ), pDep->bg_targets_cnt = 0;
		free( pDep->pFrameBG );
		free( pDep->track_tag );
		pDep->n_objs = 0;
		pDep->n_final_objs = 0;
		pDep->width = pDep->height = 0;
		pDep->cur_status = 0;
		pDep->plus_percent = 0;
	}
}

int DepthProcess( Depth * pDep, unsigned char * pRGB, unsigned short * pDepth )
{
	int total_init = pDep->frame_ignore + pDep->frame_init;
	float VD[4] = {0},tmpf;
	int i;

	// 缓存图像-------------------------------------------------
	FilterOutDepth( pDep, pDepth );
	memcpy( pDep->pFrameRGB, pRGB, pDep->frame_bytes );
	DepthRGB2Grey( pDep, pDep->pFrameGrey, pDep->pFrameRGB );
	DepthGenGrad( pDep );

	// 判定当前状态-------------------------------------------------------
	if( pDep->frame_cnt > pDep->frame_ignore ){
		if( pDep->frame_cnt > total_init )
			pDep->cur_status = DEPTH_STATUS_NORM;
		else 
			pDep->cur_status = DEPTH_STATUS_INIT;

		// 标定摄像头------------------------------------------------------
		SearchFloor( pDep );
		tmpf = DecideCamParamMin2( pDep, VD );
		if( tmpf > 0 ){
			RefineFloor(pDep);
			DecideCamParamMin2( pDep, VD );
		}

		// 检查图像质量---------------------------------------------
		CheckCamQuality( pDep );

		if( pDep->cur_status == DEPTH_STATUS_INIT ){
			if( tmpf > 0 ){
				ApplyCamParam( pDep, VD );
			}
			DepthBkgInit( pDep, pDep->frame_cnt );
		}

		// 处理图像-----------------------------------------------------------
		if( (pDep->cur_status == DEPTH_STATUS_NORM
			|| pDep->cur_status == DEPTH_STATUS_ILL)
			&& (pDep->cam_reinit_cnt == 0)
			){
			DepthDetectMotion( pDep );
			CalcAttr( pDep );
			CalIntegralFloor( pDep->pFloorHeightInte, pDep->pFloorFGInte, pDep->pFloorPrjInte, 
				pDep->pFloorAttr, pDep->floor_attr_width, pDep->floor_attr_height );
			DetectTargets( pDep, pDep->map );
			pDep->polar_targets_cnt = 0;
			for( i=0; i<pDep->floor_targets_cnt; i++ ){
				pDep->pTargets[i].children_cnt = DetectPolarTargets( pDep, i );
			}
			//SaveID( "b:\\floor_pre.txt", pDep->pFloorAttr, pDep->map, pDep->floor_attr_width, pDep->floor_attr_height );
			RefinePolarTargets( pDep );
			//SaveID( "b:\\floor_post.txt", pDep->pFloorAttr, pDep->map, pDep->floor_attr_width, pDep->floor_attr_height );

			ReverseMap( pDep );
			RefineTargets( pDep, pDep->obj_map );
			TrackTargets( pDep );
			//*/
			DepthBkgUpdate( pDep );
		}
		else{
			pDep->n_objs = 0;
		}
	}
	else
		pDep->cur_status = DEPTH_STATUS_INIT;

	if( pDep->frame_cnt < 10000 )
		pDep->frame_cnt ++;

	return pDep->n_objs;
}

int DepthChangeStatus( Depth * pDep, int status )
{
	if( status == DEPTH_STATUS_INIT ){
		pDep->frame_cnt      = 0;
		return 1;
	}
	else
		return 0;
}

void DepthRGB2Grey( Depth * pDep, unsigned char* pDFrameGrey, unsigned char * pSFrame )
{
	int i;
	short  r0,r1,r2,r3;
	short  g0,g1,g2,g3;
	short  b0,b1,b2,b3;
	short  max0,max1,max2,max3;
	for( i=0; i<pDep->frame_len; i+=4 ){
		b0 = *pSFrame;
		g0 = *(pSFrame + 1);
		r0 = *(pSFrame + 2);
		b1 = *(pSFrame + 3);
		g1 = *(pSFrame + 4);
		r1 = *(pSFrame + 5);
		b2 = *(pSFrame + 6);
		g2 = *(pSFrame + 7);
		r2 = *(pSFrame + 8);
		b3 = *(pSFrame + 9);
		g3 = *(pSFrame + 10);
		r3 = *(pSFrame + 11);
		
		// grey region------------------------------------------------------------
		*(pDFrameGrey)   = pDep->tab_r[r0] + pDep->tab_g[g0] + pDep->tab_b[b0];
		*(pDFrameGrey+1) = pDep->tab_r[r1] + pDep->tab_g[g1] + pDep->tab_b[b1];
		*(pDFrameGrey+2) = pDep->tab_r[r2] + pDep->tab_g[g2] + pDep->tab_b[b2];
		*(pDFrameGrey+3) = pDep->tab_r[r3] + pDep->tab_g[g3] + pDep->tab_b[b3];

		// normalize rgb---------------------------------------------------------
		max0 = r0 + b0 + g0;
		if( max0 < 50 )
			max0 = 50;
		if( max0 > 0 ){
			r0 = ((r0 << 8) / max0);
			g0 = ((g0 << 8) / max0);
			b0 = ((b0 << 8) / max0);
		}

		max1 = r1 + b1 + g1;
		if( max1 < 50 )
			max1 = 50;
		if( max1 > 0 ){
			r1 = ((r1 << 8) / max1);
			g1 = ((g1 << 8) / max1);
			b1 = ((b1 << 8) / max1);
		}
	
		max2 = r2 + b2 + g2;
		if( max2 < 50 )
			max2 = 50;
		if( max2 > 0 ){
			r2 = ((r2 << 8) / max2);
			g2 = ((g2 << 8) / max2);
			b2 = ((b2 << 8) / max2);
		}

		max3 = r3 + g3 + b3;
		if( max3 < 50 )
			max3 = 50;
		if( max3 > 0 ){
			r3 = ((r3 << 8) / max3);
			g3 = ((g3 << 8) / max3);
			b3 = ((b3 << 8) / max3);
		}

		*pSFrame        = (unsigned char)b0;
		*(pSFrame + 1)  = (unsigned char)g0;
		*(pSFrame + 2)  = (unsigned char)r0;
		*(pSFrame + 3)  = (unsigned char)b1;
		*(pSFrame + 4)  = (unsigned char)g1;
		*(pSFrame + 5)  = (unsigned char)r1;
		*(pSFrame + 6)  = (unsigned char)b2;
		*(pSFrame + 7)  = (unsigned char)g2;
		*(pSFrame + 8)  = (unsigned char)r2;
		*(pSFrame + 9)  = (unsigned char)b3;
		*(pSFrame + 10) = (unsigned char)g3;
		*(pSFrame + 11) = (unsigned char)r3;

		pSFrame += 12;
		pDFrameGrey += 4;
	}
}

void DepthDetectMotion(Depth* pDep)
{
	int plus,cnt_fg,bst_plus=128;
	int min_dis = 1000, min_fg = pDep->frame_len;
	int i,dis;

	// try out the fore ground-----------------------------------------------------
	bst_plus = pDep->plus_percent;
	for( i=-pDep->plus_cnt; i<=pDep->plus_cnt; i++ ){
		plus = pDep->plus_percent + i*pDep->plus_step;
		cnt_fg = DepthTryForeGround( pDep, 128 + plus );
		dis    = abs( plus );
		if(    (cnt_fg < min_fg)
			|| ( (cnt_fg == min_fg) && (dis <= min_dis) )
			){
			min_fg   = cnt_fg;
			bst_plus = plus;
			min_dis  = dis;
		}
	}

	// detect foreground update the background-------------------------------------
	DepthDetForeGround( pDep, 128 + bst_plus );
	DepthRefineFG(pDep);
	pDep->plus_percent = bst_plus;
}

int  DepthTryForeGround( Depth* pDep, int plus )
{
	BkgModel * pFrameModel, * pFrameModel1, * pFrameModelE;
	unsigned char * pFrameGrey, * pFrameGreyE;
	int bkg_cnt_grey = 0, bkg_val_grey = 0;
	int fg_total = 0;
	unsigned short real_grey;
	int model_step = pDep->samples_per_pix * pDep->try_step;
	int rgb_step = pDep->try_step * 3;

	pFrameGrey  = pDep->pFrameGrey;
	pFrameGreyE = pDep->pFrameGrey + pDep->frame_len;
	pFrameModel = pDep->pModel;
	while( pFrameGrey<pFrameGreyE ){
		// compare current pix to the bkg-------------------------------------------------
		bkg_cnt_grey = 0;
		real_grey   = (*pFrameGrey * plus) >> 7;
		pFrameModel1 = pFrameModel;
		pFrameModelE = pFrameModel + pDep->samples_per_pix;
		while(pFrameModel1 < pFrameModelE){

			bkg_val_grey = abs(pFrameModel1->grey - real_grey);

			if( bkg_val_grey <= pDep->dif_thresh_grey )
				bkg_cnt_grey ++;

			if( bkg_cnt_grey >= pDep->bg_cnt_thresh )
				break;

			pFrameModel1 += 1;
		}

		// it belongs to background--------------------------------------------------------
		if( bkg_cnt_grey < pDep->bg_cnt_thresh ){
			fg_total ++;
		}

		pFrameModel += model_step;
		pFrameGrey  += pDep->try_step;
	}
	return fg_total;
}

int  DepthDetForeGround( Depth* pDep, int plus )
{
	BkgModel * pFrameModel, * pFrameModel1, * pFrameModelE, * pFrameModel2;
	short bkg_cnt_grey = 0, bkg_val_grey = 0;
	short bkg_cnt_rgb  = 0, bkg_val_rgb  = 0;
	short bkg_cnt_dep  = 0, bkg_val_dep  = 0, bkg_cnt_fg2bg = 0, bkg_dep_accu=0, bkg_dep_accu_cnt=0;
	short bkg_cnt_grad = 0, bkg_val_grad = 0;
	short shadow_grey_thresh = 0;
	short true_grey_thresh = 0;
	int fg_total = 0;
	short real_d,real_grad;
	unsigned short * pFrameD, * pFrameDE;
	ImgAttr * pImgAttr = pDep->pImgAttr;

	short real_grey,real_r,real_g,real_b;
	unsigned char * pFrameGrey, * pFrameRGB, * pFrameGrad;
	pFrameRGB   = pDep->pFrameRGB;
	pFrameGrey  = pDep->pFrameGrey;
	pFrameGrad  = pDep->pFrameGrad;

	pFrameD     = pDep->pFrameDepth;
	pFrameDE    = pDep->pFrameDepth + pDep->frame_len;
	pFrameModel = pDep->pModel;
	true_grey_thresh   = pDep->dif_thresh_grey;
	shadow_grey_thresh = (true_grey_thresh * pDep->dif_thresh_shad) >> 7;
	while( pFrameD < pFrameDE ){
		// compare current pix to the bkg-------------------------------------------------
		bkg_dep_accu=0, bkg_dep_accu_cnt=0;
		bkg_cnt_dep  = 0, bkg_cnt_fg2bg = 0;
		bkg_cnt_grey = 0;
		bkg_cnt_rgb  = 0;
		bkg_cnt_grad = 0;

		real_grey   = (*pFrameGrey * plus) >> 7;
		real_r      = pFrameRGB[0];
		real_g      = pFrameRGB[1];
		real_b      = pFrameRGB[2];
		real_grad   = pFrameGrad[0];

		real_d      = *pFrameD;
		pFrameModel1 = pFrameModel;
		pFrameModelE = pFrameModel + pDep->samples_per_pix;
		pFrameModel2 = pFrameModel1;
		
		while(pFrameModel1 < pFrameModelE){

			if( (real_d > 0) && (pFrameModel1->d > 0) )
				bkg_val_dep = pFrameModel1->d - real_d;
			else
				bkg_val_dep = 0;

			if( bkg_val_dep > 0 ){
				bkg_dep_accu += bkg_val_dep;
				bkg_dep_accu_cnt ++;
			}

			bkg_val_grad= real_grad - pFrameModel1->grad;

			bkg_val_grey= abs((short)pFrameModel1->grey - real_grey);

			bkg_val_rgb = abs((short)pFrameModel1->r  - real_r) 
						+ abs((short)pFrameModel1->g  - real_g) 
						+ abs((short)pFrameModel1->b  - real_b);

			if( abs(bkg_val_dep) < pDep->dif_thresh_dep )
				bkg_cnt_dep ++;
			if( -bkg_val_dep > pDep->dif_thresh_dep )
				bkg_cnt_fg2bg ++;

			// adjust the grey and shadow thresh according to the grey value
			true_grey_thresh   = pDep->dif_thresh_grey;// + ((40*abs(real_grey-128))>>7);
			shadow_grey_thresh = (true_grey_thresh * pDep->dif_thresh_shad) >> 7;

			if(bkg_val_grad <= pDep->dif_thresh_grad)
				bkg_cnt_grad ++;
			if( (bkg_val_rgb <= pDep->dif_thresh_hue) || (real_grey < 60) ){
				if( abs(bkg_val_grey) <= true_grey_thresh )
					bkg_cnt_rgb ++;
				else if( (bkg_val_grey <= shadow_grey_thresh) && (bkg_val_grey >= 0) )
					bkg_cnt_grey ++;
			}
			if( (bkg_cnt_rgb >= pDep->bg_cnt_thresh)
				&& (bkg_cnt_grad>= pDep->bg_cnt_thresh)
				&& (bkg_cnt_dep >= pDep->bg_cnt_thresh)
				)
				break;

			pFrameModel1 += 1;
		}

		if( real_d > 0 )
			pImgAttr->type = PT_TYPE_DVALID;
		else
			pImgAttr->type = PT_TYPE_INVALID;

		if( bkg_cnt_dep < pDep->bg_cnt_thresh ){
			pImgAttr->type |= PT_TYPE_FG;
			if( bkg_cnt_fg2bg > (pDep->samples_per_pix>>1) )
				pImgAttr->type |= PT_TYPE_FG2BG;
			else
				pImgAttr->type |= PT_TYPE_FG_DEP;
		}
		if(bkg_cnt_grad < pDep->bg_cnt_thresh){
			pImgAttr->type |= PT_TYPE_FG;
			pImgAttr->type |= PT_TYPE_FG_GRAD;
		}
		if( bkg_cnt_rgb < pDep->bg_cnt_thresh ){
			if( bkg_cnt_grey < pDep->bg_cnt_thresh ){
				pImgAttr->type |= PT_TYPE_FG;
				pImgAttr->type |= PT_TYPE_FG_RGB;
			}
			else{
				pImgAttr->type |= PT_TYPE_FG;
				pImgAttr->type |= PT_TYPE_SHADOW;
			}
		}

		if( !(pImgAttr->type & PT_TYPE_FG) )
		{ // it belongs to background-----------------------------------------------------
			if( pImgAttr->active_cnt > 0 )
				pImgAttr->active_cnt --;
		}

		if( bkg_dep_accu_cnt > 0 )
			pImgAttr->avg_bg_dep = bkg_dep_accu / bkg_dep_accu_cnt;
		else
			pImgAttr->avg_bg_dep = 0;

		pFrameModel += pDep->samples_per_pix;
		pFrameD     ++;
		pFrameGrey  ++;
		pFrameRGB   += 3;
		pImgAttr    ++;
		pFrameGrad  ++;
	}
	return fg_total;
}//*/

void DepthBkgInit(Depth* pDep,int idx)
{
	unsigned char * pFrameGrey, * pFrameGreyE, * pFrameRGB, * pFrameGrad;
	unsigned short * pFrameDep;
	BkgModel * pFrameModel;
	idx = idx % pDep->samples_per_pix;
	pFrameRGB   = pDep->pFrameRGB;
	pFrameGrey  = pDep->pFrameGrey;
	pFrameGrad  = pDep->pFrameGrad;
	pFrameGreyE = pDep->pFrameGrey + pDep->frame_len;
	pFrameDep   = pDep->pFrameDepth;
	pFrameModel = pDep->pModel;
	while( pFrameGrey < pFrameGreyE ){
		pFrameModel[idx].grey   = *pFrameGrey;
		pFrameModel[idx].grad   = *pFrameGrad;
		pFrameModel[idx].r      = pFrameRGB[0];
		pFrameModel[idx].g      = pFrameRGB[1];
		pFrameModel[idx].b      = pFrameRGB[2];
		if( *pFrameDep )
			pFrameModel[idx].d      = *pFrameDep;
		pFrameGrey  ++;
		pFrameGrad  ++;
		pFrameRGB   += 3;
		pFrameDep   += 1;
		pFrameModel += pDep->samples_per_pix;
	}
}

void DepthBkgUpdate(Depth* pDep)
{
	int rnd,rndx,rndy,rndi,rndd;
	unsigned char * pFrameGrey, * pFrameRGB, * pFrameGrad;
	BkgModel * pFrameModel, * pFrameModel1, * pFrameModel2;
	ImgAttr * pImgAttr, * pImgAttrE;
	int u,v,i,isok;
	pFrameGrey = pDep->pFrameGrey;
	pFrameGrad = pDep->pFrameGrad;
	pFrameRGB  = pDep->pFrameRGB;
	pImgAttr   = pDep->pImgAttr;
	pImgAttrE  = pDep->pImgAttr + pDep->frame_len;
	pFrameModel = pDep->pModel;
	u=0,v=0;
	while( pImgAttr < pImgAttrE ){
		if( ((pImgAttr->type & PT_TYPE_FG) == 0 )  // 不是前景
			|| ( ((pImgAttr->type & PT_TYPE_FG_DEP) == 0) && (pImgAttr->active_cnt==0) )  // 是非深度前景，但不是活跃点
			|| ( (pImgAttr->type & PT_TYPE_FG2BG) && (pImgAttr->is_in_vir==0) )           // 不在门区域内的前景转背景
			){
			isok = 1;
			for( i=0; i<pDep->n_objs; i++ ){
				if( IsInRect( u,v, pDep->pObjs[i].region ) && 
					(    (pDep->pObjs[i].type != TARGET_TYPE_FLOOR) 
					  || ( ((pImgAttr->type & PT_TYPE_FG_DEP) == 0) && (pImgAttr->active_cnt==0) )
					  )
					){
					isok = 0;
					break;
				}
			}
			if( isok ){
				///*
				// update current pix model---------------------------------------------------
				rndi = rand()%pDep->n_subsampling;
				rndd = rand()%pDep->n_subsampling_d;
				rnd = rand()%pDep->samples_per_pix;
				pFrameModel1 = pFrameModel+rnd;
				if( (rndd == 0) && (pImgAttr->depth>0) && (!((pImgAttr->type & PT_TYPE_FG2BG)&&(pImgAttr->type & (PT_TYPE_FG_RGB|PT_TYPE_FG_GRAD)))) )
					pFrameModel1->d = pImgAttr->depth;
				if( (rndi == 0) || (pImgAttr->type & PT_TYPE_FG2BG) ){
					pFrameModel1->grey   = (*pFrameGrey);
					pFrameModel1->grad   = (*pFrameGrad);
					pFrameModel1->r      = pFrameRGB[0];
					pFrameModel1->g      = pFrameRGB[1];
					pFrameModel1->b      = pFrameRGB[2];
				}//*/
				///*
				// update neighboring pixel----------------------------------------------------
				rnd = rand()%(pDep->n_subsampling_d>>2);
				if( rnd == 0 ){
					rndx = rand()%pDep->neighbor_span - (pDep->neighbor_span>>1);
					rndy = rand()%pDep->neighbor_span - (pDep->neighbor_span>>1);
					pFrameModel1 = pFrameModel + (rndy*pDep->width + rndx)*pDep->samples_per_pix;
					if( (pFrameModel1 >= pDep->pModel) && (pFrameModel1 < pDep->pModel + pDep->frame_len*pDep->samples_per_pix) ){
						rnd = rand()%(pDep->samples_per_pix>>1);  // 只能向前一半的样本投射
						pFrameModel2 = pFrameModel1 + rnd;
						(*pFrameModel2).grey  = (*pFrameGrey);
						(*pFrameModel2).grad  = (*pFrameGrad);
						(*pFrameModel2).r     = pFrameRGB[0];
						(*pFrameModel2).g     = pFrameRGB[1];
						(*pFrameModel2).b     = pFrameRGB[2];
						if( pImgAttr->depth > 0 )
							(*pFrameModel2).d     = pImgAttr->depth;
					}
				}//*/
			}
		}
		if( u<pDep->width-1 ){
			u++;
		}
		else{
			u = 0;
			v++;
		}
		pImgAttr    ++;
		pFrameGrey  ++;
		pFrameGrad  ++;
		pFrameRGB   += 3;
		pFrameModel += pDep->samples_per_pix;
	}
}

void DepthRefineFG( Depth* pDep )
{
	ImgAttr * pImgAttr = pDep->pImgAttr, * pImgNeighbor;
	ImgAttr * pImgAttrE= pDep->pImgAttr + pDep->frame_len;
	int x, y, i, j, k, nextId,m;
	int vw=pDep->width;
	int vh=pDep->height;
	int idOfs[4],ids[4];
	int is_fg;
	short * map   = pDep->map;

	idOfs[0] = -vw-1;
	idOfs[1] = -vw;
	idOfs[2] = -vw+1;
	idOfs[3] = -1;

	// to start, every id value maps to itself
	nextId = 1;
	for( i = 0; i<=pDep->max_blobs; i++){
		map[i] = i;
	}
	
	// scan first pixel as a special case
	is_fg = !(pImgAttr->type & PT_TYPE_FG);
	if ( is_fg ){
		pImgAttr->id = nextId++;
	}
	else
		pImgAttr->id = 0;
	pImgAttr ++;
	
	// scan rest of first row as a special case
	for(x=1; x<vw; x++){
		is_fg = !(pImgAttr->type & PT_TYPE_FG);
		if ( is_fg ){
			pImgNeighbor = pImgAttr - 1;
			j = map[pImgNeighbor->id];
			if (j > 0){
				pImgAttr->id = j;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr ++;
	}
	
	// scan rest of rows
	for(y=1; y<vh; y++){
		is_fg = !(pImgAttr->type & PT_TYPE_FG);
		// check first pixel of row as a special case
		if ( is_fg ){
			m = 0;
			pImgNeighbor = pImgAttr - vw;
			i = map[pImgNeighbor->id];
			if( i > m )
				m = i;

			pImgNeighbor = pImgAttr - vw + 1;
			i = map[pImgNeighbor->id];
			if( i > m )
				m = i;
		
			if (m>0){
				pImgAttr->id = m;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr ++;
	
	    // now check the 'middle' of the row
		for(x=1; x<vw-1; x++){
			is_fg = !(pImgAttr->type & PT_TYPE_FG);
			if ( is_fg ){
				j = 0;
				// find the max neighbor
				for( i = 0; i<4; i++){
					pImgNeighbor = pImgAttr + idOfs[i];
					k = pImgNeighbor->id;
					ids[i] = map[k];
					if (ids[i] > j) 
						j = ids[i];
				}
				if (j > 0){
					for( i = 0; i<4; i++){
						if (ids[i]==0 || ids[i]==j) 
							continue;
						for(k=1; k<nextId; k++){
							if (map[k]==ids[i])
								map[k] = j;
						}
					}
					pImgAttr->id = j;
				}
				else{
					pImgAttr->id = nextId++;
				}
			}
			else
				pImgAttr->id = 0;
			pImgAttr ++;
		}
	
		// finally, we can check the last pixel of the row as a special case
		is_fg = !(pImgAttr->type & PT_TYPE_FG);
		if ( is_fg ){
			m = 0;
			pImgNeighbor = pImgAttr - vw - 1;
			i = map[pImgNeighbor->id];
			if( i > m )
				m = i;

			pImgNeighbor = pImgAttr - vw;
			i = map[pImgNeighbor->id];
			if( i > m )
				m = i;

			pImgNeighbor = pImgAttr - 1;
			i = map[pImgNeighbor->id];
			if( i > m )
				m = i;
			
			if (m>0){
				pImgAttr->id = m;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr++;
		
		if (nextId > pDep->max_blobs){
			printf("nextId exceeded!\n");
			return;
		}
	}
	//SaveIDImg( "b:\\it.txt", pDep->pImgAttr, map, pDep->width, pDep->height );
	for( i=0; i<nextId; i++ ){
		pDep->bg_targets[i].rect.left = pDep->width;
		pDep->bg_targets[i].rect.right= 0;
		pDep->bg_targets[i].rect.top  = pDep->height;
		pDep->bg_targets[i].rect.bottom= 0;
		pDep->bg_targets[i].mass = 0;
	}
	pImgAttr = pDep->pImgAttr;
	for( y=0; y<pDep->height; y++ ){
		for( x=0; x<pDep->width; x++ ){
			if (pImgAttr[x].id > 0){
				i = map[pImgAttr[x].id];
				pImgAttr[x].id = i;
				i -= 1;
				if( pDep->bg_targets[i].rect.left > x )
					pDep->bg_targets[i].rect.left = x;
				if( pDep->bg_targets[i].rect.right < x )
					pDep->bg_targets[i].rect.right = x + 1;
				if( pDep->bg_targets[i].rect.top > y )
					pDep->bg_targets[i].rect.top = y;
				if( pDep->bg_targets[i].rect.bottom < y )
					pDep->bg_targets[i].rect.bottom = y + 1;
				pDep->bg_targets[i].mass ++;
			}
		}
		pImgAttr += pDep->width;
	}
	for( i=0; i<nextId-1; i++ ){
		if( (pDep->bg_targets[i].rect.left > 0)
			&& (pDep->bg_targets[i].rect.right < pDep->width)
			&& (pDep->bg_targets[i].rect.top > 0)
			&& (pDep->bg_targets[i].rect.bottom < pDep->height)
			&& (pDep->bg_targets[i].mass > 0)
			)
			map[i+1] = -1;
	}
	pImgAttr = pDep->pImgAttr;
	while( pImgAttr < pImgAttrE ){
		if( map[pImgAttr->id] < 0 ){
			pImgAttr->type |= PT_TYPE_FG;
			pImgAttr->type |= PT_TYPE_FG_EXT;
		}
		pImgAttr ++;
	}
	//SaveIDImg( "b:\\it_post.txt", pDep->pImgAttr, map, pDep->width, pDep->height );
}

void SearchFloor( Depth * pDep )
{
	int edge = 10;
	int v_edge_stride = edge * pDep->width;
	int u,v,cu,cv;
	float VD[3],height;
	float ptc[3],ptu[3],ptd[3],ptl[3],ptr[3],ptul[3],ptur[3],ptdl[3],ptdr[3];
	ImgPlaneAttr  * pImgPlaneAttr = pDep->pImgPlaneAttr;
	unsigned short * pDepC  = pDep->pFrameDepth;
	unsigned short * pDepU  = pDepC - v_edge_stride;
	unsigned short * pDepD  = pDepC + v_edge_stride;
	unsigned short * pDepL  = pDepC - edge;
	unsigned short * pDepR  = pDepC + edge;
	unsigned short * pDepUL = pDepU - edge;
	unsigned short * pDepUR = pDepU + edge;
	unsigned short * pDepDL = pDepD - edge;
	unsigned short * pDepDR = pDepD + edge;
	bool isok;
	float thresh;
	Plane * pPlane = pDep->pPlanes;
	int plane_cnt = 0;
	
	thresh = 0.01f;
	cu = pDep->width  >> 1;
	cv = pDep->height >> 1;
	for( v = 0; v < pDep->height; v++ ){
		for( u = 0; u < pDep->width; u++ ){
			pDepU = pDepC - v_edge_stride;
			pDepD = pDepC + v_edge_stride;
			pDepL = pDepC - edge;
			pDepR = pDepC + edge;
			pDepUL= pDepU - edge;
			pDepUR= pDepU + edge;
			pDepDL= pDepD - edge;
			pDepDR= pDepD + edge;

			ptc[2] = ptu[2] = ptd[2] = ptl[2] = ptr[2] = 0;
			ptul[2] = ptur[2] = ptdl[2] = ptdr[2] = 0;
			CalcPT( ptc, *pDepC, u-cu, v-cv, pDep->fu, pDep->fv );

			if( v >= edge ){
				CalcPT( ptu, *pDepU, u-cu, v-edge-cv, pDep->fu, pDep->fv );
				if( u >= edge )
					CalcPT( ptul, *pDepUL, u-edge-cu, v-edge-cv, pDep->fu, pDep->fv );
				if( u < pDep->width - edge )
					CalcPT( ptur, *pDepUR, u+edge-cu, v-edge-cv, pDep->fu, pDep->fv );
			}
			if( u >= edge )
				CalcPT( ptl, *pDepL, u-edge-cu, v-cv, pDep->fu, pDep->fv );
			if( u < pDep->width-edge )
				CalcPT( ptr, *pDepR, u+edge-cu, v-cv, pDep->fu, pDep->fv );
			if( v < pDep->height - edge ){
				CalcPT( ptd, *pDepD, u-cu, v+edge-cv, pDep->fu, pDep->fv );
				if( u >= edge )
					CalcPT( ptdl, *pDepDL, u-edge-cu, v+edge-cv, pDep->fu, pDep->fv );
				if( u < pDep->width - edge )
					CalcPT( ptdr, *pDepDR, u+edge-cu, v+edge-cv, pDep->fu, pDep->fv );
			}

			pDepC ++;

			isok = false;
			if( ptc[2] ){
				//*
				if( ptul[2] && ptdl[2] && ptr[2] ){
					isok = CalcDirection( ptc, ptul, ptdl, ptr, VD, thresh );
				}
				if( !isok && ptl[2] && ptur[2] && ptdr[2] ){
					isok = CalcDirection( ptc, ptl, ptur, ptdr, VD, thresh );
				}
				if( !isok && ptul[2] && ptur[2] && ptd[2] ){
					isok = CalcDirection( ptc, ptul, ptur, ptd, VD, thresh );
				}
				if( !isok && ptl[2] && ptur[2] && ptdr[2] ){
					isok = CalcDirection( ptc, ptl, ptur, ptdr, VD, thresh );
				}//*/
			}

			pImgPlaneAttr->x = ptc[0];
			pImgPlaneAttr->y = ptc[1];
			pImgPlaneAttr->z = ptc[2];
			pImgPlaneAttr->plane_type = PLANE_TYPE_NO;
			pImgPlaneAttr->height = 0;
			if( isok ){ // 四个点在一个平面
				if( VD[2] > 0 ){
					VD[0] = -VD[0];
					VD[1] = -VD[1];
					VD[2] = -VD[2];
				}
				height = ptc[0]*VD[0] + ptc[1]*VD[1] + ptc[2]*VD[2];
				if( height < 0 )
					height = -height;
				pImgPlaneAttr->height = height;
				if( (VD[0]>-0.25)&&(VD[0]<0.25)&&(VD[1]<0)&&(height>pDep->cam_h_low)&&(height<pDep->cam_h_high)  ){
					pImgPlaneAttr->plane_type = PLANE_TYPE_UP;
				}
			}
			pImgPlaneAttr++;
		}
	}
}

bool CalcDirection( float * CP1, float * CP2, float * CP3, float * CP4, float * VD, float thresh )
{
	float VA[3], VB[3], VC[3], VC_Norm, VD_Norm, plane_eva;
	float dis_1_2, dis_1_3, dis_1_4;

	dis_1_2 = CP1[2] - CP2[2];
	dis_1_3 = CP1[2] - CP3[2];
	dis_1_4 = CP1[2] - CP4[2];
	if( dis_1_2 < 0 )
		dis_1_2 = -dis_1_2;
	if( dis_1_3 < 0 )
		dis_1_3 = -dis_1_3;
	if( dis_1_4 < 0 )
		dis_1_4 = -dis_1_4;

	if( dis_1_2 < dis_1_3 )
		dis_1_2 = dis_1_3;
	if( dis_1_2 < dis_1_4 )
		dis_1_2 = dis_1_4;

	if( dis_1_2 > 500 )
		return false;

	// compute P1->P2
	VA[0] = CP2[0] - CP1[0];
	VA[1] = CP2[1] - CP1[1];
	VA[2] = CP2[2] - CP1[2];
	// compute P1->P3
	VB[0] = CP3[0] - CP1[0];
	VB[1] = CP3[1] - CP1[1];
	VB[2] = CP3[2] - CP1[2];
	// compute P1->P4
	VC[0] = CP4[0] - CP1[0];
	VC[1] = CP4[1] - CP1[1];
	VC[2] = CP4[2] - CP1[2];

	// compute (P1->P2)*(P1->P3)
	VD[0] = VA[1]*VB[2] - VA[2]*VB[1];
	VD[1] = VA[2]*VB[0] - VA[0]*VB[2];
	VD[2] = VA[0]*VB[1] - VA[1]*VB[0];

	VD_Norm = (float)sqrt( VD[0]*VD[0] + VD[1]*VD[1] + VD[2]*VD[2] );
	VC_Norm = (float)sqrt( VC[0]*VC[0] + VC[1]*VC[1] + VC[2]*VC[2] );

	VD[0] /= VD_Norm;
	VD[1] /= VD_Norm;
	VD[2] /= VD_Norm;

	VC[0] /= VC_Norm;
	VC[1] /= VC_Norm;
	VC[2] /= VC_Norm;

	plane_eva = VC[0]*VD[0] + VC[1]*VD[1] + VC[2]*VD[2];
	if( plane_eva < 0 )
		plane_eva = -plane_eva;

	if( plane_eva < thresh )
		return true;
	else
		return false;
}

void CalcPT( float pt[3], unsigned short dep, int u, int v, int fu, int fv )
{
	pt[2] = dep;
	pt[0] = u*pt[2] / fu;
	pt[1] = v*pt[2] / fv;
}

void SearchPlane( Depth * pDep )
{
	int edge = pDep->height >> 5;
	int v_edge_stride = edge * pDep->width;
	int i,j,u,v,cu,cv;
	float VD[3];
	float ptc[3],ptu[3],ptd[3],ptl[3],ptr[3],ptul[3],ptur[3],ptdl[3],ptdr[3];
	ImgPlaneAttr  * pImgPlaneAttr = pDep->pImgPlaneAttr;
	unsigned short * pDepC  = pDep->pFrameDepth;
	unsigned short * pDepU  = pDepC - v_edge_stride;
	unsigned short * pDepD  = pDepC + v_edge_stride;
	unsigned short * pDepL  = pDepC - edge;
	unsigned short * pDepR  = pDepC + edge;
	unsigned short * pDepUL = pDepU - edge;
	unsigned short * pDepUR = pDepU + edge;
	unsigned short * pDepDL = pDepD - edge;
	unsigned short * pDepDR = pDepD + edge;
	bool isok;
	float thresh,prj,prj2,vpt[3];
	Plane * pPlane = pDep->pPlanes;
	int plane_cnt = 0,idx,cnt;
	
	thresh = 0.01f;
	cu = pDep->width  >> 1;
	cv = pDep->height >> 1;
	for( v = 0; v < pDep->height; v++ ){
		for( u = 0; u < pDep->width; u++ ){
			pDepU = pDepC - v_edge_stride;
			pDepD = pDepC + v_edge_stride;
			pDepL = pDepC - edge;
			pDepR = pDepC + edge;
			pDepUL= pDepU - edge;
			pDepUR= pDepU + edge;
			pDepDL= pDepD - edge;
			pDepDR= pDepD + edge;

			ptc[2] = ptu[2] = ptd[2] = ptl[2] = ptr[2] = 0;
			ptul[2] = ptur[2] = ptdl[2] = ptdr[2] = 0;
			CalcPT( ptc, *pDepC, u-cu, v-cv, pDep->fu, pDep->fv );

			if( v >= edge ){
				CalcPT( ptu, *pDepU, u-cu, v-edge-cv, pDep->fu, pDep->fv );
				if( u >= edge )
					CalcPT( ptul, *pDepUL, u-edge-cu, v-edge-cv, pDep->fu, pDep->fv );
				if( u < pDep->width - edge )
					CalcPT( ptur, *pDepUR, u+edge-cu, v-edge-cv, pDep->fu, pDep->fv );
			}
			if( u >= edge )
				CalcPT( ptl, *pDepL, u-edge-cu, v-cv, pDep->fu, pDep->fv );
			if( u < pDep->width-edge )
				CalcPT( ptr, *pDepR, u+edge-cu, v-cv, pDep->fu, pDep->fv );
			if( v < pDep->height - edge ){
				CalcPT( ptd, *pDepD, u-cu, v+edge-cv, pDep->fu, pDep->fv );
				if( u >= edge )
					CalcPT( ptdl, *pDepDL, u-edge-cu, v+edge-cv, pDep->fu, pDep->fv );
				if( u < pDep->width - edge )
					CalcPT( ptdr, *pDepDR, u+edge-cu, v+edge-cv, pDep->fu, pDep->fv );
			}

			pDepC ++;

			isok = false;
			if( ptc[2] ){
				//*
				if( ptul[2] && ptdl[2] && ptr[2] ){
					isok = CalcDirection( ptc, ptul, ptdl, ptr, VD, thresh );
				}
				if( !isok && ptl[2] && ptur[2] && ptdr[2] ){
					isok = CalcDirection( ptc, ptl, ptur, ptdr, VD, thresh );
				}
				if( !isok && ptul[2] && ptur[2] && ptd[2] ){
					isok = CalcDirection( ptc, ptul, ptur, ptd, VD, thresh );
				}
				if( !isok && ptl[2] && ptur[2] && ptdr[2] ){
					isok = CalcDirection( ptc, ptl, ptur, ptdr, VD, thresh );
				}//*/

				//if( !isok && ptul[2] && ptl[2] && ptu[2] ){
				//	isok = CalcDirection( ptc, ptu, ptul, ptl, VD, thresh );
				//}
				//if( !isok && ptur[2] && ptr[2] && ptu[2] ){
				//	isok = CalcDirection( ptc, ptu, ptur, ptr, VD, thresh );
				//}
				//if( !isok && ptdl[2] && ptl[2] && ptd[2] ){
				//	isok = CalcDirection( ptc, ptd, ptdl, ptl, VD, thresh );
				//}
				//if( !isok && ptdr[2] && ptr[2] && ptd[2] ){
				//	isok = CalcDirection( ptc, ptd, ptdr, ptr, VD, thresh );
				//}
			}

			if( isok ){ // 四个点在一个平面
				if( VD[2] > 0 ){
					VD[0] = -VD[0];
					VD[1] = -VD[1];
					VD[2] = -VD[2];
				}
				if( (VD[0]<0.2) && (VD[1]<0) ) // 大致朝上的点
					pImgPlaneAttr->plane_type = PLANE_TYPE_UP;
				else
					pImgPlaneAttr->plane_type = PLANE_TYPE_OTHER;

				// try to add the point to a plane
				for( i=0; i<plane_cnt; i++ ){
					cnt = 0;
					for( j=0; j<pPlane[i].pt_cnt; j++ ){
						vpt[0] = ptc[0] - pPlane[i].pts[j][0];
						vpt[1] = ptc[1] - pPlane[i].pts[j][1];
						vpt[2] = ptc[2] - pPlane[i].pts[j][2];
						prj = (float)sqrt(vpt[0]*vpt[0] + vpt[1]*vpt[1] + vpt[2]*vpt[2]);
						vpt[0] /= prj, vpt[1] /= prj, vpt[2] /= prj;
						prj = vpt[0]*VD[0] + vpt[1]*VD[1] + vpt[2]*VD[2];
						if( prj < 0 )
							prj = -prj;
						prj2= vpt[0]*pPlane[i].dir[j][0] + vpt[1]*pPlane[i].dir[j][1] + vpt[2]*pPlane[i].dir[j][2];
						if( prj < 0.01 ){
							cnt ++;
						}
					}
					if( cnt > (pPlane[i].pt_cnt>>3) ){  // add to the plane
						if( pPlane[i].pt_cnt < MAX_PLANE_PTS ){
							idx = pPlane[i].pt_cnt;
							pPlane[i].pt_cnt ++;
						}
						else{
							idx = rand()%MAX_PLANE_PTS;
						}
						pPlane[i].pts[idx][0] = ptc[0];
						pPlane[i].pts[idx][1] = ptc[1];
						pPlane[i].pts[idx][2] = ptc[2];
						pPlane[i].dir[idx][0] = VD[0];
						pPlane[i].dir[idx][1] = VD[1];
						pPlane[i].dir[idx][2] = VD[2];
						pPlane[i].pts_uv[idx][0] = u;
						pPlane[i].pts_uv[idx][1] = v;
						pPlane[i].total ++;
						break;
					}
				}
				if( (i==plane_cnt) && (plane_cnt<pDep->max_blobs) ){    // new plane
					pPlane[plane_cnt].pts[0][0] = ptc[0];
					pPlane[plane_cnt].pts[0][1] = ptc[1];
					pPlane[plane_cnt].pts[0][2] = ptc[2];
					pPlane[plane_cnt].dir[0][0] = VD[0];
					pPlane[plane_cnt].dir[0][1] = VD[1];
					pPlane[plane_cnt].dir[0][2] = VD[2];
					pPlane[plane_cnt].pts_uv[0][0] = u;
					pPlane[plane_cnt].pts_uv[0][1] = v;
					pPlane[plane_cnt].pt_cnt = 1;
					pPlane[plane_cnt].total  = 1;
					plane_cnt ++;
				}
			}
			else{
				pImgPlaneAttr->plane_type = PLANE_TYPE_NO;
			}
			pImgPlaneAttr ++;
		}
	}

	// fitler out small planes------------------------------------------
	pDep->plane_cnt = 0;
	for( i=0; i<plane_cnt; i++ ){
		if( pPlane[i].total > 1000 ){
			memcpy( pPlane + pDep->plane_cnt, pPlane + i, sizeof(pPlane[0]) );
			pDep->plane_cnt ++;
		}
	}
}

float  DecideCamParamMin2( Depth * pDep, float * rVD )
{
	int cnt;
	int cu,cv;
	unsigned short * pDepCenter = pDep->pFrameDepth;
	ImgPlaneAttr * pImg = pDep->pImgPlaneAttr;
	ImgPlaneAttr * pImgE= pDep->pImgPlaneAttr + pDep->frame_len;

	cu = pDep->width  >> 1;
	cv = pDep->height >> 1;

	cnt = 0;
	while( pImg < pImgE ){
		if( pImg->plane_type == PLANE_TYPE_UP ){
			pDep->x[cnt] = pImg->x;
			pDep->y[cnt] = pImg->y;
			pDep->z[cnt] = pImg->z;
			cnt ++;
		}
		pImg ++;
	}

	if( cnt > 10 )
		FitPlane( pDep->x, pDep->y, pDep->z, cnt, pDep->plane );  // ax+by+cz=d拟合
	else
		return -1;
	//tmpf = sqrt( pDep->plane[0]*pDep->plane[0] + pDep->plane[1]*pDep->plane[1] + pDep->plane[2]*pDep->plane[2] );
	if( pDep->plane[3] < 0 ){
		rVD[3] = - pDep->plane[3];
		rVD[0] = pDep->plane[0];
		rVD[1] = pDep->plane[1];
		rVD[2] = pDep->plane[2];
	}
	else{
		rVD[3] = pDep->plane[3];
		rVD[0] = -pDep->plane[0];
		rVD[1] = -pDep->plane[1];
		rVD[2] = -pDep->plane[2];
	}

	return rVD[3];
}

void ApplyCamParam( Depth * pDep, float * VD )
{
	int u,v;
	float uu,vv,uc,vc;	
	int * pXCoeff, * pYCoeff, * pZCoeff;
	float XW[3],YW[3],ZW[3],DotProduct,Norm;
	int learn_rate = pDep->vd_learn_rate;

	// 要进行对该向量执行学习过程
	if( pDep->cur_status == DEPTH_STATUS_INIT ){
		learn_rate = 700;
		pDep->VD[0] = (pDep->VD[0]*(1000-learn_rate) + VD[0]*learn_rate) / 1000;
		pDep->VD[1] = (pDep->VD[1]*(1000-learn_rate) + VD[1]*learn_rate) / 1000;
		pDep->VD[2] = (pDep->VD[2]*(1000-learn_rate) + VD[2]*learn_rate) / 1000;
		pDep->VD[3] = (pDep->VD[3]*(1000-learn_rate) + VD[3]*learn_rate) / 1000;
		VD[0] = pDep->VD[0];
		VD[1] = pDep->VD[1];
		VD[2] = pDep->VD[2];
		VD[3] = pDep->VD[3];

		pDep->cam_height = (int)VD[3];
		uc = (float)(pDep->width  >> 1);
		vc = (float)(pDep->height >> 1);

		/* Initialize the XW YW ZW arrays */
		ZW[0] = (float)VD[0]; ZW[1] = (float)VD[1]; ZW[2] = (float)VD[2];
		// 由于VD已经是Normalize过的，所以无需再Normalize一次
		Norm = (float)sqrt((float)ZW[0]*ZW[0] + (float)ZW[1]*ZW[1] + (float)ZW[2]*ZW[2]);
		ZW[0] = ZW[0]/Norm; ZW[1] = ZW[1]/Norm; ZW[2] = ZW[2]/Norm;

		XW[0] = 1.0; XW[1] = 0; XW[2] = 0;
		DotProduct = XW[0]*ZW[0] + XW[1]*ZW[1] + XW[2]*ZW[2];
		XW[0] = XW[0] - DotProduct*ZW[0];
		XW[1] = XW[1] - DotProduct*ZW[1];
		XW[2] = XW[2] - DotProduct*ZW[2];
		Norm = (float)sqrt((float)XW[0]*XW[0] + (float)XW[1]*XW[1] + (float)XW[2]*XW[2]);
		XW[0] = XW[0]/Norm; XW[1] = XW[1]/Norm; XW[2] = XW[2]/Norm;
	
		YW[0] = ZW[1]*XW[2] - ZW[2]*XW[1];
		YW[1] = ZW[2]*XW[0] - ZW[0]*XW[2];
		YW[2] = ZW[0]*XW[1] - ZW[1]*XW[0];

		Inv( XW, YW, ZW, pDep->invX, pDep->invY, pDep->invZ );
	
		/* Initialize the XYZ constant matrixes */
		pXCoeff = pDep->pXCoeff;
		pYCoeff = pDep->pYCoeff;
		pZCoeff = pDep->pZCoeff;
		for(v=0;v<pDep->height;v++)
		{
			for(u=0;u<pDep->width;u++)
			{
				uu =((u-uc))/pDep->fu;
				vv =((v-vc))/pDep->fv;
			
				/* Maybe need to be scaled up */
				*pXCoeff = (int)((XW[0]*uu + XW[1]*vv + XW[2])*1024);//pow(2,CONST_SCALE_BIT);
				*pYCoeff = (int)((YW[0]*uu + YW[1]*vv + YW[2])*1024);//pow(2,CONST_SCALE_BIT);
				*pZCoeff = (int)((ZW[0]*uu + ZW[1]*vv + ZW[2])*1024);//pow(2,CONST_SCALE_BIT);
			
				pXCoeff ++;
				pYCoeff ++;
				pZCoeff ++;
			}
		}
	}
}

int DepthRectDif( Rect lrect, Rect rrect, Rect * pcommon )
{
	Rect common;
	int larea,rarea,comarea,w,h;
	if( lrect.left > rrect.left )
		common.left = lrect.left;
	else
		common.left = rrect.left;

	if( lrect.right < rrect.right )
		common.right = lrect.right;
	else
		common.right = rrect.right;

	if( lrect.top > rrect.top )
		common.top = lrect.top;
	else
		common.top = rrect.top;

	if( lrect.bottom < rrect.bottom )
		common.bottom = lrect.bottom;
	else
		common.bottom = rrect.bottom;
	w = common.right - common.left;
	h = common.bottom - common.top;
	if( (w<0) || (h<0) )
		comarea = 0;
	else
		comarea = w * h;
	larea = (lrect.right - lrect.left) * (lrect.bottom - lrect.top);
	rarea = (rrect.right - rrect.left) * (rrect.bottom - rrect.top);
	if( larea > rarea )
		larea = rarea;

	if( pcommon )
		*pcommon = common;

	if( larea > 0 )
		return comarea * 100 / larea;
	else
		return 100;
}

int DepthOuterDif( Rect lrect, Rect rrect )
{
	Rect common;
	int larea,rarea,comarea,w,h;
	if( lrect.left > rrect.left )
		common.left = lrect.left;
	else
		common.left = rrect.left;

	if( lrect.right < rrect.right )
		common.right = lrect.right;
	else
		common.right = rrect.right;

	if( lrect.top > rrect.top )
		common.top = lrect.top;
	else
		common.top = rrect.top;

	if( lrect.bottom < rrect.bottom )
		common.bottom = lrect.bottom;
	else
		common.bottom = rrect.bottom;
	w = common.right - common.left;
	h = common.bottom - common.top;
	if( (w<0) || (h<0) )
		comarea = 0;
	else
		comarea = w * h;
	larea = (lrect.right - lrect.left) * (lrect.bottom - lrect.top);
	rarea = (rrect.right - rrect.left) * (rrect.bottom - rrect.top);
	if( larea < rarea ){
		larea = rarea;
	}

	if( larea > 0 )
		return comarea * 100 / larea;
	else
		return 100;}

int DepthInnerDif( Rect lrect, Rect rrect )
{
	Rect common;
	int larea,rarea,comarea,w,h;
	if( lrect.left > rrect.left )
		common.left = lrect.left;
	else
		common.left = rrect.left;

	if( lrect.right < rrect.right )
		common.right = lrect.right;
	else
		common.right = rrect.right;

	if( lrect.top > rrect.top )
		common.top = lrect.top;
	else
		common.top = rrect.top;

	if( lrect.bottom < rrect.bottom )
		common.bottom = lrect.bottom;
	else
		common.bottom = rrect.bottom;
	w = common.right - common.left;
	h = common.bottom - common.top;
	if( (w<0) || (h<0) )
		comarea = 0;
	else
		comarea = w * h;
	larea = (lrect.right - lrect.left) * (lrect.bottom - lrect.top);
	rarea = (rrect.right - rrect.left) * (rrect.bottom - rrect.top);
	if( larea > rarea ){
		if( (larea>>2) < rarea )
			larea = rarea;
		else
			larea = larea >> 2;
	}

	if( larea > 0 )
		return comarea * 100 / larea;
	else
		return 100;
}

int UpdateRegion( Depth* pDep )
{
	int i,j,isok=1,cu,cv,idx;
	float real_u,real_v,real_d;
	cu = pDep->width >> 1;
	cv = pDep->height >> 1;
	for( i=0; i<REGION_MAX_CNT; i++ ){
		if( pDep->regions[i].n_pt > 2 ){
			pDep->regions[i].status = REGION_STATUS_NORMAL;
			for( j=0; j<pDep->regions[i].n_pt; j++ ){
				real_u = (float)((pDep->regions[i].u[j] - cu) / pDep->fu);
				real_v = (float)((pDep->regions[i].v[j] - cv) / pDep->fv);
				real_d = -pDep->VD[3]/(pDep->VD[0]*real_u + pDep->VD[1]*real_v + pDep->VD[2]);
				idx = pDep->regions[i].v[j]*pDep->width+pDep->regions[i].u[j];
				pDep->regions[i].x[j] = (int)(pDep->pXCoeff[idx] * real_d) >> 10;
				pDep->regions[i].y[j] = (int)(pDep->pYCoeff[idx] * real_d) >> 10;
			}
		}
		else
			pDep->regions[i].status = REGION_STATUS_UNSET;
	}
	if( pDep->valid_region.n_pt > 2 ){
		pDep->valid_region.status = REGION_STATUS_NORMAL;
		for( j=0; j<pDep->valid_region.n_pt; j++ ){
			real_u = (float)((pDep->valid_region.u[j] - cu) / pDep->fu);
			real_v = (float)((pDep->valid_region.v[j] - cv) / pDep->fv);
			real_d = -pDep->VD[3]/(pDep->VD[0]*real_u + pDep->VD[1]*real_v + pDep->VD[2]);
			idx = pDep->valid_region.v[j]*pDep->width+pDep->valid_region.u[j];
			pDep->valid_region.x[j] = (int)(pDep->pXCoeff[idx] * real_d) >> 10;
			pDep->valid_region.y[j] = (int)(pDep->pYCoeff[idx] * real_d) >> 10;
		}
	}
	else
		pDep->valid_region.status = REGION_STATUS_UNSET;
	return isok;
}

void SaveFloorAttr( Depth* pDep )
{
	int u,v;
	FILE * pfile = fopen("d:\\floor.txt","w");
	FloorAttr * pFloorAttr = pDep->pFloorAttr;
	for( v=0; v<pDep->floor_attr_width; v++ ){
		for( u=0; u<pDep->floor_attr_width; u++ ){
			fprintf(pfile,"%8d",pFloorAttr[u].max_height/*,pFloorAttr[u].prj_cnt*/);
		}
		pFloorAttr += pDep->floor_attr_width;
		fprintf(pfile,"\n");
	}
	fclose(pfile);
}

//float inv_u,inv_v,inv_d;
//int inv_uu,inv_vv;
void CalcAttr( Depth* pDep )
{
	unsigned short * pFrameDep = pDep->pFrameDepth;
	short * edge_up, * edge_left, * edge_right, * edge_bottom;
	int * pZCoeff, * pXCoeff, * pYCoeff, dfg_cnt, nxt_dfg;
	ImgAttr * pImgAttr;
	FloorAttr * pFloorAttr;
	int u,v,i,idx_y,idx_x,offset;
	float ptc[3],height;
	int uc,vc;
	uc=pDep->width >> 1;
	vc=pDep->height >> 1;
	pXCoeff = pDep->pXCoeff;
	pYCoeff = pDep->pYCoeff;
	pZCoeff = pDep->pZCoeff;
	pImgAttr = pDep->pImgAttr;
	pFloorAttr = pDep->pFloorAttr;
	memset( pFloorAttr, 0, sizeof(FloorAttr)*pDep->floor_attr_width*pDep->floor_attr_height );
	edge_up = pDep->edge_up;
	edge_left = pDep->edge_left;
	edge_right = pDep->edge_right;
	edge_bottom = pDep->edge_bottom;

	memset( edge_up, 255, sizeof(short)*pDep->width );
	memset( edge_left, 255, sizeof(short)*pDep->height );
	memset( edge_right, 0, sizeof(short)*pDep->height );
	memset( edge_bottom, 0, sizeof(short)*pDep->width );

	for( i=0; i<REGION_MAX_CNT; i++ ){
		pDep->regions[i].dfg_cnt = 0;
	}

	dfg_cnt = 0;
	for( v=0; v<pDep->height; v++ ){
		for( u=0; u<pDep->width; u++ ){
			pImgAttr[u].id = 0;
			pImgAttr[u].is_in_vir = 0;
			pImgAttr[u].bg_height = 0;
			if( pFrameDep[u] != 0 ){
				// 计算边缘点坐标-------------------------------------------------------
				if( edge_up[u] < 0 )
					edge_up[u] = v;
				if( edge_left[v] < 0 )
					edge_left[v] = u;
				edge_right[v] = u;
				edge_bottom[u] = v;

				// 计算区域的点数属性---------------------------------------------------
				if( pImgAttr[u].type & (PT_TYPE_FG_DEP | PT_TYPE_FG2BG) ){
					for( i=0; i<REGION_MAX_CNT; i++ ){
						if( pDep->regions[i].type == REGION_TYPE_VIO ){
							if( IsPtInPolygon( u,v,pDep->regions[i].u, pDep->regions[i].v, pDep->regions[i].n_pt ) ){
								pDep->regions[i].dfg_cnt ++;
								pImgAttr[u].is_in_vir = 1;
							}
						}
					}
				}

				CalcPT( ptc, pFrameDep[u], u-uc, v-vc, pDep->fu, pDep->fv );
				height = ptc[0]*pDep->VD[0] + ptc[1]*pDep->VD[1] + ptc[2]*pDep->VD[2] + pDep->VD[3];

				// 计算图像点属性-------------------------------------------------------
				pImgAttr[u].depth  = pFrameDep[u];
				pImgAttr[u].x      = ((pXCoeff[u] * pFrameDep[u]) >> 10);
				pImgAttr[u].y      = ((pYCoeff[u] * pFrameDep[u]) >> 10);
				pImgAttr[u].height = ((pZCoeff[u] * pFrameDep[u]) >> 10) + pDep->cam_height;
				if( pImgAttr[u].avg_bg_dep > 0 )
					pImgAttr[u].bg_height = ((pZCoeff[u] * pImgAttr[u].avg_bg_dep) >> 10) + pDep->cam_height;
				/*
				inv_u = pImgAttr[u].x * pDep->invX[0] + pImgAttr[u].y * pDep->invX[1] + (pImgAttr[u].height-pDep->cam_height)*pDep->invX[2];
				inv_v = pImgAttr[u].x * pDep->invY[0] + pImgAttr[u].y * pDep->invY[1] + (pImgAttr[u].height-pDep->cam_height)*pDep->invY[2];
				inv_d = pImgAttr[u].x * pDep->invZ[0] + pImgAttr[u].y * pDep->invZ[1] + (pImgAttr[u].height-pDep->cam_height)*pDep->invZ[2];
				inv_uu = inv_u*pDep->fu/inv_d+(pDep->width>>1);
				inv_vv = inv_v*pDep->fv/inv_d+(pDep->height>>1);
				//*/

				if( pImgAttr[u].height < 0 )
					pImgAttr[u].height = 0;
				if( pImgAttr[u].height < pDep->floor_hthresh )
					pImgAttr[u].type |= PT_TYPE_FLOOR;

				if( (pImgAttr[u].x < (pDep->floor_x_span>>1)) 
				    && (pImgAttr[u].x >= -(pDep->floor_x_span>>1)) 
					&& (pImgAttr[u].y < pDep->floor_y_base + pDep->floor_y_span) 
					&& (pImgAttr[u].y >= pDep->floor_y_base)
					){
					idx_y = (pImgAttr[u].y - pDep->floor_y_base) / pDep->floor_unit;
					idx_x = (pImgAttr[u].x + (pDep->floor_x_span>>1)) / pDep->floor_unit;
					idx_y = pDep->floor_attr_height - 1 - idx_y;
					offset = idx_y * pDep->floor_attr_width + idx_x;
					pImgAttr[u].offset = offset;
					if(pImgAttr[u].type & PT_TYPE_FG_DEP){   // 只有深度前景点才投射
						dfg_cnt ++;
						pFloorAttr[offset].id = -1;
						if( pFloorAttr[offset].prj_cnt == 0 ){
							pFloorAttr[offset].min_height = pImgAttr[u].height;
							pFloorAttr[offset].max_height = pImgAttr[u].height;
							pFloorAttr[offset].max_u = u;
							pFloorAttr[offset].max_v = v;
							pFloorAttr[offset].min_u = u;
							pFloorAttr[offset].min_v = v;
						}
						else{
							if( pFloorAttr[offset].min_height > pImgAttr[u].height ){
								pFloorAttr[offset].min_height = pImgAttr[u].height;
								pFloorAttr[offset].min_u = u;
								pFloorAttr[offset].min_v = v;
							}
							if( pFloorAttr[offset].max_height < pImgAttr[u].height ){
								pFloorAttr[offset].max_height = pImgAttr[u].height;
								pFloorAttr[offset].max_u = u;
								pFloorAttr[offset].max_v = v;
							}
						}
						pFloorAttr[offset].prj_cnt ++;
					}
				}
				else{
					pImgAttr[u].offset = -1;
				}
			}
			else{
				pImgAttr[u].x      = -1;
				pImgAttr[u].y      = -1;
				pImgAttr[u].height = -1;
				pImgAttr[u].type   = PT_TYPE_INVALID;
				pImgAttr[u].offset = -1;
			}
		}
		pXCoeff += pDep->width;
		pYCoeff += pDep->width;
		pZCoeff += pDep->width;
		pImgAttr+= pDep->width;
		pFrameDep += pDep->width;
	}
	pDep->his_DFG[pDep->cur_dfg] = dfg_cnt;
	nxt_dfg = (pDep->cur_dfg + 1)%MAX_GREY_HISTORY;
	pDep->dfg_dif_percent = (abs(pDep->his_DFG[pDep->cur_dfg] - pDep->his_DFG[nxt_dfg]) << 7) / (pDep->width*pDep->height);
	pDep->cur_dfg = nxt_dfg;
}

void  RefineFloor( Depth * pDep )
{
	ImgPlaneAttr * pImgAttr = pDep->pImgPlaneAttr;
	ImgPlaneAttr * pImgAttrE= pDep->pImgPlaneAttr + pDep->frame_len;
	int cu = pDep->width>>1;
	int cv = pDep->height>>1;
	int * pXCoeff = pDep->pXCoeff;
	int * pYCoeff = pDep->pYCoeff;
	int * pZCoeff = pDep->pZCoeff;
	if( pDep->plane[2] > 0 ){
		pDep->plane[0] = -pDep->plane[0];
		pDep->plane[1] = -pDep->plane[1];
		pDep->plane[2] = -pDep->plane[2];
		pDep->plane[3] = -pDep->plane[3];
	}
	// collect the height of each pixel------------------------------------------
	pDep->up_pt_cnt = 0;
	pDep->min_height = 0;
	while( pImgAttr < pImgAttrE ){
		if( (pImgAttr->plane_type == PLANE_TYPE_NO) || (pImgAttr->plane_type == PLANE_TYPE_UP) ){
			pImgAttr->height = ( pDep->plane[0]*pImgAttr->x + pDep->plane[1]*pImgAttr->y + pDep->plane[2]*pImgAttr->z - pDep->plane[3] );
			if( pImgAttr->height > 50 )
				pImgAttr->plane_type = PLANE_TYPE_NO;
			else{
				pImgAttr->plane_type = PLANE_TYPE_UP;
				pDep->up_pt_cnt ++;
			}
		}
		pImgAttr+= 1;
	}
}

void CalIntegralFloor( long long* pInte, int * pFloorFGInte, int * pFloorPrjInte, FloorAttr* pFloor, int width, int height )
{
	int i,j;
	long long partialsum;
	int partialsum_fg;
	int partialsum_prj;
	pInte[0] = pFloor[0].max_height;
	pFloorFGInte[0] = 0;
	pFloorPrjInte[0] = pFloor[0].prj_cnt;
	for(i=1;i<width;i++)
	{
		pInte[i] = pInte[i-1] + (long long)pFloor[i].max_height;
		pFloorFGInte[i] = pFloorFGInte[i-1] + ( pFloor[i].max_height > 0 ? 1 : 0 );
		pFloorPrjInte[i] = pFloorPrjInte[i-1] + pFloor[i].prj_cnt;
	}
	for(i=1;i<height;i++)
	{
		partialsum = 0;
		partialsum_fg = 0;
		partialsum_prj = 0;
		for(j=0;j<width;j++)
		{
			partialsum += (long long)pFloor[i*width+j].max_height;
			partialsum_fg += (pFloor[i*width+j].max_height > 0) ? 1 : 0;
			partialsum_prj += pFloor[i*width+j].prj_cnt;

			pInte[i*width+j] = pInte[(i-1)*width+j] + partialsum;
			pFloorFGInte[i*width+j] = pFloorFGInte[(i-1)*width+j] + partialsum_fg;
			pFloorPrjInte[i*width+j] = pFloorPrjInte[(i-1)*width+j] + partialsum_prj;
		}
	}
}

void DoubleFilter( Depth * pDep )
{
	/*
	FloorAttr * pFloorAttr = pDep->pFloorAttr + (pDep->floor_filter_w2+1)*pDep->floor_attr_width + pDep->floor_filter_w2 + 1;
	int width_h = (pDep->floor_filter_w<<1)+1;
	int width_h2= (pDep->floor_filter_w2<<1)+1;
	int area_h = width_h  * width_h;
	int area_h2= width_h2 * width_h2;
	long long * pInteUL = pDep->pFloorHeightInte 
		                + (pDep->floor_filter_w2+1-pDep->floor_filter_w-1)*pDep->floor_attr_width 
		                + pDep->floor_filter_w2 + 1 - pDep->floor_filter_w - 1;
	long long * pInteUR = pInteUL + width_h;
	long long * pInteDR = pInteUR + width_h * pDep->floor_attr_width;
	long long * pInteDL = pInteDR - width_h;
	long long * pInteUL2= pDep->pFloorHeightInte;
	long long * pInteUR2= pInteUL2+ width_h2;
	long long * pInteDR2= pInteUR2+ width_h2*pDep->floor_attr_width;
	long long * pInteDL2= pInteDR2- width_h2;
	int real_h = pDep->floor_attr_height - width_h2;
	int real_w = pDep->floor_attr_width  - width_h2;
	int y,x;

	for( y=0; y<real_h; y++ ){
		for( x=0; x<real_w; x++ ){
			pFloorAttr[x].filter_h = (pInteDR[x]  - pInteUR[x]  - pInteDL[x]  + pInteUL[x] ) / area_h;
			pFloorAttr[x].filter_h2= (pInteDR2[x] - pInteUR2[x] - pInteDL2[x] + pInteUL2[x]) / area_h2;
		}
		pFloorAttr += pDep->floor_attr_width;
		pInteUL += pDep->floor_attr_width;
		pInteUR += pDep->floor_attr_width;
		pInteDR += pDep->floor_attr_width;
		pInteDL += pDep->floor_attr_width;
		pInteUL2+= pDep->floor_attr_width;
		pInteUR2+= pDep->floor_attr_width;
		pInteDR2+= pDep->floor_attr_width;
		pInteDL2+= pDep->floor_attr_width;
	}
	//*/
}

bool DetectTargets( Depth * pDep, short * map )
{
	FloorAttr * pFloorAttr = pDep->pFloorAttr;
	FloorAttr * pFloorAttrE= pDep->pFloorAttr + pDep->floor_attr_width * pDep->floor_attr_height;
	int x, y, i, j, k, nextId,m;
	int ids[4];
	int vw=pDep->floor_attr_width;
	int vh=pDep->floor_attr_height;
	int nPixels=vw*vh;
	int idOfs[4];
	int id_top     = pDep->max_blobs - pDep->n_objs;
	FloorTarget * pTargets = pDep->pTargets;
	PolarTarget * pPolarTargets = pDep->pPolarTargets;
	int is_fg;
	int abh_th = pDep->floor_hthresh;

	idOfs[0] = -vw-1;
	idOfs[1] = -vw;
	idOfs[2] = -vw+1;
	idOfs[3] = -1;

	// to start, every id value maps to itself
	nextId = 1;
	for( i = 0; i<=pDep->max_blobs; i++){
		map[i] = i;
	}
	
	// scan first pixel as a special case
	is_fg = (pFloorAttr->max_height > abh_th) && (pFloorAttr->id == -1);  // 高度足够且是深度前景的投影
	if ( is_fg ){
		pFloorAttr->id = nextId++;
	}
	else{
	    pFloorAttr->id = 0;
	}
	pFloorAttr ++;
	
	// scan rest of first row as a special case
	for(x=1; x<vw; x++){

		is_fg = (pFloorAttr->max_height > abh_th) && (pFloorAttr->id == -1);
		if ( is_fg ){
			j = (pFloorAttr - 1)->id;
			if (j > 0){
				pFloorAttr->id = j;
			}
			else{
				pFloorAttr->id = nextId++;
			}
		}
		else{
			pFloorAttr->id = 0;
		}

		pFloorAttr ++;
	}
	
	// scan rest of rows
	for(y=1; y<vh; y++){

		is_fg = (pFloorAttr->max_height > abh_th) && (pFloorAttr->id == -1);
		// check first pixel of row as a special case
		if ( is_fg ){
			i = map[(pFloorAttr - vw)->id];
			j = map[(pFloorAttr - vw + 1)->id];
		
			if (j>i)
				m = j;
			else
				m = i;

			if (m>0){
				pFloorAttr->id = m;
			}
			else{
				pFloorAttr->id = nextId++;
			}
		}
		else{
			pFloorAttr->id = 0;
		}

		pFloorAttr ++;
	
	    // now check the 'middle' of the row
		for(x=1; x<vw-1; x++){

			is_fg = (pFloorAttr->max_height > abh_th) && (pFloorAttr->id == -1);
			if ( is_fg ){
				j = 0;
				// find the max neighbor
				for( i = 0; i<4; i++){
					k = (pFloorAttr + idOfs[i])->id;
					ids[i] = map[k];
					if (ids[i] > j) 
						j = ids[i];
				}
				if (j > 0){
					for( i = 0; i<4; i++){
						if (ids[i]==0 || ids[i]==j) 
							continue;
						for(k=1; k<nextId; k++){
							if (map[k]==ids[i])
								map[k] = j;
						}
					}
					pFloorAttr->id = j;
				}
				else{
					pFloorAttr->id = nextId++;
				}
			}
			else{
				pFloorAttr->id = 0;
			}

			pFloorAttr ++;
		}
	
		// finally, we can check the last pixel of the row as a special case
		is_fg = (pFloorAttr->max_height > abh_th) && (pFloorAttr->id == -1);
		if ( is_fg ){
			i = (pFloorAttr - vw - 1)->id;
			j = (pFloorAttr - vw)->id;
			if (j>i) 
				i = j;
			
			j = (pFloorAttr - 1)->id;
			if (j>i)
				i = j;
			
			if (i>0){
				pFloorAttr->id = i;
			}
			else{
				pFloorAttr->id = nextId++;
			}
		}
		else{
			pFloorAttr->id = 0;
		}
		pFloorAttr++;
		
		if (nextId > pDep->max_blobs){
			printf("nextId exceeded!\n");
			return false;
		}
	}

	//SaveID( pDep->pFloorAttr, map, pDep->floor_attr_width, pDep->floor_attr_height );
	
	// pass 2 - collect the targets-------------------------------------------------
	memset( pTargets, 0, sizeof(FloorTarget)*pDep->max_blobs );
	memset( pPolarTargets, 0, sizeof(PolarTarget)*pDep->max_blobs );
	pFloorAttr = pDep->pFloorAttr;
	for( y=0; y<pDep->floor_attr_height; y++ ){
		for( x=0; x<pDep->floor_attr_width; x++ ){
			if (pFloorAttr[x].id > 0){
				i = map[pFloorAttr[x].id];
				assert(i>0);
				pFloorAttr[x].id = i;
				i -= 1;
				if( pTargets[i].max_height < pFloorAttr[x].max_height ){
					pTargets[i].max_height = pFloorAttr[x].max_height;
					pTargets[i].x = x;
					pTargets[i].y = y;
				}
				if( pTargets[i].mass == 0 ){
					pTargets[i].rect.left = pTargets[i].rect.right = x;
					pTargets[i].rect.top  = pTargets[i].rect.bottom= y;
				}
				else{
					if( pTargets[i].rect.left > x )
						pTargets[i].rect.left = x;
					if( pTargets[i].rect.right < x + 1 )
						pTargets[i].rect.right = x + 1;
					if( pTargets[i].rect.top > y )
						pTargets[i].rect.top = y;
					if( pTargets[i].rect.bottom < y + 1 )
						pTargets[i].rect.bottom = y + 1;
				}
				if( pFloorAttr[x].max_height < (abh_th << 1) )
					pTargets[i].floor_mass ++;
				//dis_up    = abs( pFloorAttr[x].max_v - pDep->edge_up[pFloorAttr[x].max_u] );
				//dis_left  = abs( pFloorAttr[x].max_u - pDep->edge_left[pFloorAttr[x].max_v] );
				//dis_right = abs( pFloorAttr[x].max_u - pDep->edge_right[pFloorAttr[x].max_v] );
				//if( ((pFloorAttr[x].max_height / pFloorAttr[x].prj_cnt < 150)
				//	&& ((dis_up < 10) || (dis_left < 10) || (dis_right < 10)))
				//	|| ( (pFloorAttr[x].max_height>1800) && (pFloorAttr[x].prj_cnt>50) )
				//	)
				//	pTargets[i].wall_mass += 1;//pFloorAttr[x].prj_cnt;
				pTargets[i].mass += 1;//pFloorAttr[x].prj_cnt;
			}
		}
		pFloorAttr += pDep->floor_attr_width;
	}

	// merge the targets------------------------------------------------------------------------------------------------
	memset( map, 0, sizeof(short)*(pDep->max_blobs+1) );
	pDep->floor_targets_cnt = 0;
	for( i=0; i<nextId; i++ ){
		if( pTargets[i].mass > pDep->min_floor_blob_size ){
			pTargets[pDep->floor_targets_cnt].max_height = pTargets[i].max_height;
			pTargets[pDep->floor_targets_cnt].x          = pTargets[i].x;//pTargets[i].x * pDep->floor_unit - (pDep->floor_x_span>>1);
			pTargets[pDep->floor_targets_cnt].y          = pTargets[i].y;//pTargets[i].y * pDep->floor_unit + pDep->floor_y_base;
			pTargets[pDep->floor_targets_cnt].mass       = pTargets[i].mass;
			pTargets[pDep->floor_targets_cnt].floor_mass = pTargets[i].floor_mass;
			pTargets[pDep->floor_targets_cnt].rect       = pTargets[i].rect;
			pTargets[pDep->floor_targets_cnt].id         = i + 1;
			pDep->floor_targets_cnt ++;
			
			// 重新分配ID号，使其ID连续
			map[i+1] = pDep->floor_targets_cnt;
		}
		else
			map[i+1] = 0;
	}
	return true;
}

int DetectPolarTargets( Depth * pDepth, int region_idx )
{
	int y_stride,x,y,i,j;
	int real_pix = 0;
	int width = pDepth->floor_attr_width;
	int height= pDepth->floor_attr_height;
	int stride = pDepth->cr_stride;
	int c_w = pDepth->center_radius;
	int r_w = (pDepth->round_radiius - pDepth->center_radius)>>1;
	int cr_w= pDepth->round_radiius;
	long long * pInte = pDepth->pFloorHeightInte;
	int * pFGInte = pDepth->pFloorFGInte;
	int * pPrjInte= pDepth->pFloorPrjInte;
	//int areac = c_w * c_w;
	int areaf = r_w * r_w;
	int arean = r_w * c_w;
	int hh_stride = r_w * width;
	int idxr,idxl,idxc,idxt,idxlb,idxrb,idxrr,idxtr,idxll,idxcl,idxct,idxtt,idxcc,idxtc,idxlc,idxrc;
	long long fgc,fgl,fgr,fgtl,fgtr,fgt,fgb,fgbl,fgbr;
	int areac,areal,arear,areatl,areatr,areat,areab,areabl,areabr;
	int avgh_c,avgh_l,avgh_r,avgh_t,avgh_b,avgh_tl,avgh_tr,avgh_bl,avgh_br;
	int h_th,inc_w;
	PolarTarget * pTargets = pDepth->pPolarTargets + pDepth->polar_targets_cnt;
	int tgt_cnt = 0, cnt = 0;
	FloorTarget * pFloorTgt = pDepth->pTargets + region_idx;
	Rect region = pDepth->pTargets[region_idx].rect;

	pTargets[0].rect.left = pFloorTgt->x - (c_w>>1);
	pTargets[0].rect.right= pFloorTgt->x + (c_w>>1);
	pTargets[0].rect.top  = pFloorTgt->y - (c_w>>1);
	pTargets[0].rect.bottom=pFloorTgt->y + (c_w>>1);
	pTargets[0].avg_height =-1;
	pTargets[0].root = region_idx;
	tgt_cnt = 1;

	if( pDepth->pTargets[region_idx].max_height > 1000 ){
		if( region.left <= 0 )
			region.left = 1;
		if( region.top <= 0 )
			region.top = 1;

		y_stride = stride * width;

		idxc = (region.top -1) * width - 1;
		idxt = idxc + c_w;
		idxl = idxc + c_w * width;
		idxr = idxl + c_w;

		idxct= idxc - hh_stride;
		idxtt= idxt - hh_stride;
		idxlb= idxl + hh_stride;
		idxrb= idxr + hh_stride;
		idxcl= idxc - r_w;
		idxll= idxl - r_w;
		idxtr= idxt + r_w;
		idxrr= idxr + r_w;

		idxcc= idxct - r_w;
		idxtc= idxtt + r_w;
		idxlc= idxlb - r_w;
		idxrc= idxrb + r_w;

		for( y = region.top; y <= region.bottom - c_w; y+=stride ){
			for( x = region.left; x <= region.right - c_w; x+=stride ){
				// calc the fg stuff----------------------------------------------
				///*
				fgc   = pInte[idxr+x] - pInte[idxl+x] - pInte[idxt+x] + pInte[idxc+x];
				areac = pFGInte[idxr+x] - pFGInte[idxl+x] - pFGInte[idxt+x] + pFGInte[idxc+x];
				if( areac == 0 )
					areac = 1;
				if( y > r_w ){
					fgt   = pInte[idxt+x] - pInte[idxc+x] - pInte[idxtt+x] + pInte[idxct+x];
					areat = pFGInte[idxt+x] - pFGInte[idxc+x] - pFGInte[idxtt+x] + pFGInte[idxct+x];
					if( areat == 0 )
						areat = 1;
				}
				else{
					fgt = 0;
					areat = 1;
				}
				if( y + c_w + r_w < height ){
					fgb = pInte[idxrb+x] - pInte[idxlb+x] - pInte[idxr+x] + pInte[idxl+x];
					areab = pFGInte[idxrb+x] - pFGInte[idxlb+x] - pFGInte[idxr+x] + pFGInte[idxl+x];
					if( areab == 0 )
						areab = 1;
				}
				else{
					fgb = 0;
					areab = 1;
				}
				if( x > r_w ){
					fgl = pInte[idxl+x] - pInte[idxll+x] - pInte[idxc+x] + pInte[idxcl+x];
					areal = pFGInte[idxl+x] - pFGInte[idxll+x] - pFGInte[idxc+x] + pFGInte[idxcl+x];
					if( areal == 0 )
						areal = 1;
				}
				else{
					fgl = 0;
					areal = 1;
				}
				if( x + c_w + r_w < width ){
					fgr = pInte[idxrr+x] - pInte[idxr+x] - pInte[idxtr+x] + pInte[idxt+x];
					arear = pFGInte[idxrr+x] - pFGInte[idxr+x] - pFGInte[idxtr+x] + pFGInte[idxt+x];
					if( arear == 0 )
						arear = 1;
				}
				else{
					fgr = 0;
					arear = 1;
				}
				if( (y > r_w) && (x > r_w) ){
					fgtl = pInte[idxc+x] - pInte[idxcl+x] - pInte[idxct+x] + pInte[idxcc+x];
					areatl = pFGInte[idxc+x] - pFGInte[idxcl+x] - pFGInte[idxct+x] + pFGInte[idxcc+x];
					if( areatl == 0 )
						areatl = 1;
				}
				else{
					fgtl = 0;
					areatl = 1;
				}
				if( (y > r_w) && (x + c_w + r_w < width) ){
					fgtr = pInte[idxtr+x] - pInte[idxt+x] - pInte[idxtt+x] + pInte[idxtt+x];
					areatr = pFGInte[idxtr+x] - pFGInte[idxt+x] - pFGInte[idxtt+x] + pFGInte[idxtt+x];
					if( areatr == 0 )
						areatr = 1;
				}
				else{
					fgtr = 0;
					areatr = 1;
				}
				if( (y + c_w + r_w < height) && (x > r_w) ){
					fgbl = pInte[idxlb+x] - pInte[idxlc+x] - pInte[idxl+x] + pInte[idxll+x];
					areabl = pFGInte[idxlb+x] - pFGInte[idxlc+x] - pFGInte[idxl+x] + pFGInte[idxll+x];
					if( areabl == 0 )
						areabl = 1;
				}
				else{
					fgbl = 0;
					areabl = 1;
				}
				if( (y + c_w + r_w < height) && (x + c_w + r_w < width) ){
					fgbr = pInte[idxrc+x] - pInte[idxrb+x] - pInte[idxrr+x] + pInte[idxr+x];
					areabr = pFGInte[idxrc+x] - pFGInte[idxrb+x] - pFGInte[idxrr+x] + pFGInte[idxr+x];
					if( areabr == 0 )
						areabr = 1;
				}
				else{
					fgbr = 0;
					areabr = 1;
				}

				assert( fgc >= 0 );
				assert( fgt >= 0 );
				assert( fgb >= 0 );
				assert( fgl >= 0 );
				assert( fgr >= 0 );
				assert( fgtl >= 0 );
				assert( fgtr >= 0 );
				assert( fgbl >= 0 );
				assert( fgbr >= 0 );

				avgh_c = (int)(fgc / areac);

				avgh_t= (int)(fgt / areat);
				avgh_b= (int)(fgb / areab);
				avgh_l= (int)(fgl / areal);
				avgh_r= (int)(fgr / arear);

				avgh_tl= (int)(fgtl / areatl);
				avgh_tr= (int)(fgtr / areatr);
				avgh_bl= (int)(fgbl / areabl);
				avgh_br= (int)(fgbr / areabr);

				h_th = avgh_c * pDepth->head_body_th / 100;
				if( (avgh_c - avgh_t  > h_th)
				 && (avgh_c - avgh_b  > h_th)
				 && (avgh_c - avgh_l  > h_th)
				 && (avgh_c - avgh_r  > h_th)
				 && (avgh_c - avgh_tl > h_th)
				 && (avgh_c - avgh_tr > h_th)
				 && (avgh_c - avgh_bl > h_th)
				 && (avgh_c - avgh_br > h_th)
				 && (areac*128 > c_w*c_w*70)
				 ){
					 pTargets[tgt_cnt].rect.left = x;
					 pTargets[tgt_cnt].rect.right= x + c_w;
					 pTargets[tgt_cnt].rect.top  = y;
					 pTargets[tgt_cnt].rect.bottom = y + c_w;
					 pTargets[tgt_cnt].avg_height = avgh_c;
					 for( i=0; i<tgt_cnt; i++ ){
						 if( RectDistance( pTargets[i].rect, pTargets[tgt_cnt].rect ) == 0 ){
							 pTargets[i].rect = MergeRect( pTargets[i].rect, pTargets[tgt_cnt].rect );
							 if( pTargets[i].avg_height == -1 ){
								 pTargets[i].avg_height = pTargets[tgt_cnt].avg_height;
							 }
							 break;
						 }
					 }
					 if( i == tgt_cnt ){
						 pTargets[i].root = region_idx;
						 tgt_cnt ++;
					 }
				}
			}
			idxc += y_stride, idxt += y_stride, idxl += y_stride, idxr += y_stride;
			idxct+= y_stride, idxtt+= y_stride, idxcl+= y_stride, idxll+= y_stride;
			idxtr+= y_stride, idxrr+= y_stride, idxlb+= y_stride, idxrb+= y_stride;
			idxcc+= y_stride, idxtc+= y_stride, idxlc+= y_stride, idxrc+= y_stride;
		}
	}

	// refine the detected objs -------------------------------------------------
	assert( tgt_cnt > 0 );
	if( tgt_cnt > 1 ){
		cnt = 0;
		for( i=0; i<tgt_cnt; i++ ){
			pTargets[cnt] = pTargets[i];

			inc_w = pTargets[cnt].rect.right - pTargets[cnt].rect.left;
			inc_w = (cr_w - inc_w) >> 1;
			pTargets[cnt].rect.right += inc_w;
			pTargets[cnt].rect.left  -= inc_w;

			inc_w = pTargets[cnt].rect.bottom - pTargets[cnt].rect.top;
			inc_w = (cr_w - inc_w) >> 1;
			pTargets[cnt].rect.bottom += inc_w;
			pTargets[cnt].rect.top  -= inc_w;

			if( pTargets[cnt].rect.left < region.left )
				pTargets[cnt].rect.left = region.left;
			if( pTargets[cnt].rect.right >= region.right )
				pTargets[cnt].rect.right = region.right;
			if( pTargets[cnt].rect.top < region.top )
				pTargets[cnt].rect.top = region.top;
			if( pTargets[cnt].rect.bottom >= region.bottom )
				pTargets[cnt].rect.bottom = region.bottom;

			inc_w = (cr_w + c_w) >> 1;

			if( (pTargets[cnt].rect.bottom - pTargets[cnt].rect.top < inc_w)
				&& (pTargets[cnt].rect.right - pTargets[cnt].rect.left < inc_w)
				)
				continue;

			// 消除个头过小的目标------------------------------------------------------

			// merge near objs---------------------------------------------------------
			for( j=0; j<cnt; j++ ){
				if( RectDistance( pTargets[cnt].rect, pTargets[j].rect ) == 0 ){
					if( pTargets[cnt].avg_height > pTargets[j].avg_height )
						pTargets[j] = pTargets[cnt];
					break;
				}
			}
			if( j==cnt ){
				cnt ++;
			}
		}
	}
	else
		cnt = tgt_cnt;

	pDepth->polar_targets_cnt += cnt;
	return cnt;
}

char path[128];
void RefinePolarTargets( Depth * pDepth )
{
	FloorTarget * pFloorTargets = pDepth->pTargets;
	PolarTarget * pPolarTargets = pDepth->pPolarTargets;
	PolarQueue  * pPolarQueue   = pDepth->pPolarQueue;
	FloorAttr   * pFloorAttr    = pDepth->pFloorAttr;
	ImgAttr     * pImgAttr      = pDepth->pImgAttr;
	ImgAttr     * pImgAttrT;
	int idx,idxc,idx_polar,idx_polar2;
	int root_id,ntail;
	int i,j,k,x,y,iter_cnt,bound_cnt;
	int id_change = 0;
	int queue_len = (pDepth->floor_attr_width+pDepth->floor_attr_height)*2;
	int new_dis,cj,ck;
	// init the points in the polar queue-----------------------------
	for( i=0; i<pDepth->polar_targets_cnt; i++ ){
		assert( pFloorTargets[pPolarTargets[i].root].children_cnt > 0 );
		root_id = pFloorTargets[pPolarTargets[i].root].id;
		if( pPolarTargets[i].rect.left < 0 )
			pPolarTargets[i].rect.left = 0;
		if( pPolarTargets[i].rect.right > pDepth->width )
			pPolarTargets[i].rect.right = pDepth->width;
		if( pPolarTargets[i].rect.top < 0 )
			pPolarTargets[i].rect.top = 0;
		if( pPolarTargets[i].rect.bottom > pDepth->height )
			pPolarTargets[i].rect.bottom = pDepth->height;

		cj = (pPolarTargets[i].rect.bottom + pPolarTargets[i].rect.top)>>1;
		ck = (pPolarTargets[i].rect.right + pPolarTargets[i].rect.left)>>1;
		pPolarTargets[i].mass = 0;
		idxc = pPolarTargets[i].rect.top * pDepth->floor_attr_width;
		for( j=pPolarTargets[i].rect.top; j<pPolarTargets[i].rect.bottom; j++ ){
			for( k=pPolarTargets[i].rect.left; k<pPolarTargets[i].rect.right; k++ ){
				if( pFloorAttr[idxc+k].id == root_id ){
					pFloorAttr[idxc+k].polar_id = i + 1;
					pFloorAttr[idxc+k].polar_dis= abs( j - cj );
					if( pFloorAttr[idxc+k].polar_dis < abs( k - ck ) )
						pFloorAttr[idxc+k].polar_dis = abs( k - ck );
					pPolarTargets[i].mass ++;
				}
			}
			idxc += pDepth->floor_attr_width;
		}

		idxc = pPolarTargets[i].rect.top * pDepth->floor_attr_width;
		pPolarQueue[i].head = pPolarQueue[i].tail = 0;
		// first row--------------------------------------------------------------------------------
		for( k=pPolarTargets[i].rect.left; k<pPolarTargets[i].rect.right; k++ ){
			if( pFloorAttr[idxc+k].id == root_id ){
				pPolarQueue[i].queue[pPolarQueue[i].tail].x = k;
				pPolarQueue[i].queue[pPolarQueue[i].tail].y = pPolarTargets[i].rect.top;
				pPolarQueue[i].tail = (pPolarQueue[i].tail + 1) % queue_len;
			}
		}
		idxc += pDepth->floor_attr_width;
		// middle left-------------------------------------------------------------------------------
		for( j=pPolarTargets[i].rect.top+1; j<pPolarTargets[i].rect.bottom-1; j++ ){
			for( k = pPolarTargets[i].rect.left; k<pPolarTargets[i].rect.right; k+= pPolarTargets[i].rect.right - pPolarTargets[i].rect.left - 1 ){
				if( pFloorAttr[idxc+k].id == root_id ){
					pPolarQueue[i].queue[pPolarQueue[i].tail].x = k;
					pPolarQueue[i].queue[pPolarQueue[i].tail].y = j;
					pPolarQueue[i].tail = (pPolarQueue[i].tail + 1) % queue_len;
				}
			}
			idxc += pDepth->floor_attr_width;
		}
		// last row----------------------------------------------------------------------------------
		for( k=pPolarTargets[i].rect.left; k<pPolarTargets[i].rect.right; k++ ){
			if( pFloorAttr[idxc+k].id == root_id ){
				pPolarQueue[i].queue[pPolarQueue[i].tail].x = k;
				pPolarQueue[i].queue[pPolarQueue[i].tail].y = pPolarTargets[i].rect.bottom-1;
				pPolarQueue[i].tail = (pPolarQueue[i].tail + 1) % queue_len;
			}
		}
	}
	
	//SaveID( "b:\\floor_middle.txt", pDepth->pFloorAttr, pDepth->map, pDepth->floor_attr_width, pDepth->floor_attr_height );
	// do the iteration-----------------------------------------------
	//*
	iter_cnt = 0;
	do{
		id_change = 0;
		idx_polar = 0;
		for( i = 0; i<pDepth->floor_targets_cnt; i++ ){
			root_id = pFloorTargets[i].id;
			for( j=0; j<pFloorTargets[i].children_cnt; j++ ){
				ntail = pPolarQueue[idx_polar].tail;
				for( k=pPolarQueue[idx_polar].head; k!=pPolarQueue[idx_polar].tail; k=(k+1)%queue_len ){
					x = pPolarQueue[idx_polar].queue[k].x;
					y = pPolarQueue[idx_polar].queue[k].y;
					idxc = y * pDepth->floor_attr_width + x;
					bound_cnt = 0;
					if( y > 0 ){
						// upper left
						if( x > 0 ){
							idx = idxc - pDepth->floor_attr_width - 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								if( pFloorAttr[idx].polar_id == 0){
									pPolarQueue[idx_polar].queue[ntail].x = x - 1;
									pPolarQueue[idx_polar].queue[ntail].y = y - 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									ntail = (ntail + 1)%queue_len;
									id_change ++;
								}
								else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
									bound_cnt ++;
							}
						}
						// upper
						idx = idxc - pDepth->floor_attr_width;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							if( pFloorAttr[idx].polar_id == 0 ){
								pPolarQueue[idx_polar].queue[ntail].x = x;
								pPolarQueue[idx_polar].queue[ntail].y = y - 1;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								ntail = (ntail + 1)%queue_len;
								id_change ++;
							}
							else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
								bound_cnt ++;
						}
						// upper right
						if( x < pDepth->floor_attr_width - 1 ){
							idx = idxc - pDepth->floor_attr_width + 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								if( pFloorAttr[idx].polar_id == 0 ){
									pPolarQueue[idx_polar].queue[ntail].x = x + 1;
									pPolarQueue[idx_polar].queue[ntail].y = y - 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									ntail = (ntail + 1)%queue_len;
									id_change ++;
								}
								else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
									bound_cnt ++;
							}
						}
					}
					// left
					if( x > 0 ){
						idx = idxc - 1;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							if( pFloorAttr[idx].polar_id == 0 ){
								pPolarQueue[idx_polar].queue[ntail].x = x - 1;
								pPolarQueue[idx_polar].queue[ntail].y = y;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								ntail = (ntail + 1)%queue_len;
								id_change ++;
							}
							else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
								bound_cnt ++;
						}
					}
					// right
					if( x < pDepth->floor_attr_width - 1 ){
						idx = idxc + 1;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							if( pFloorAttr[idx].polar_id == 0 ){
								pPolarQueue[idx_polar].queue[ntail].x = x + 1;
								pPolarQueue[idx_polar].queue[ntail].y = y;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								ntail = (ntail + 1)%queue_len;
								id_change ++;
							}
							else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
								bound_cnt ++;
						}
					}
					if( y < pDepth->floor_attr_height - 1 ){
						// lower left
						if( x > 0 ){
							idx = idxc + pDepth->floor_attr_width - 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								if( pFloorAttr[idx].polar_id == 0 ){
									pPolarQueue[idx_polar].queue[ntail].x = x - 1;
									pPolarQueue[idx_polar].queue[ntail].y = y + 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									ntail = (ntail + 1)%queue_len;
									id_change ++;
								}
								else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
									bound_cnt ++;
							}
						}
						// lower
						idx = idxc + pDepth->floor_attr_width;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							if( pFloorAttr[idx].polar_id == 0 ){
								pPolarQueue[idx_polar].queue[ntail].x = x;
								pPolarQueue[idx_polar].queue[ntail].y = y + 1;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								ntail = (ntail + 1)%queue_len;
								id_change ++;
							}
							else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
								bound_cnt ++;
						}
						// lower right
						if( x < pDepth->floor_attr_width - 1 ){
							idx = idxc + pDepth->floor_attr_width + 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								if( pFloorAttr[idx].polar_id == 0 ){
									pPolarQueue[idx_polar].queue[ntail].x = x + 1;
									pPolarQueue[idx_polar].queue[ntail].y = y + 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									ntail = (ntail + 1)%queue_len;
									id_change ++;
								}
								else if( pFloorAttr[idx].polar_id != idx_polar + 1 )
									bound_cnt ++;
							}
						}
					}
					if( bound_cnt > 0 ){
						pPolarQueue[idx_polar].queue[ntail].x = x;
						pPolarQueue[idx_polar].queue[ntail].y = y;
						ntail = (ntail + 1)%queue_len;
					}
				}
				pPolarQueue[idx_polar].head = k;
				pPolarQueue[idx_polar].tail = ntail;
				idx_polar ++;
			}
		}
		iter_cnt ++;
	}while(id_change);

	//sprintf(path,"b:\\floor_middle_%d.txt",iter_cnt-1);
	//SaveID( path, pDepth->pFloorAttr, pDepth->map, pDepth->floor_attr_width, pDepth->floor_attr_height );

	//*
	do{
		id_change = 0;
		idx_polar = 0;
		for( i = 0; i<pDepth->floor_targets_cnt; i++ ){
			root_id = pFloorTargets[i].id;
			for( j=0; j<pFloorTargets[i].children_cnt; j++ ){
				ntail = pPolarQueue[idx_polar].tail;
				for( k=pPolarQueue[idx_polar].head; k!=pPolarQueue[idx_polar].tail; k=(k+1)%queue_len ){
					id_change ++;
					x = pPolarQueue[idx_polar].queue[k].x;
					y = pPolarQueue[idx_polar].queue[k].y;
					idxc = y * pDepth->floor_attr_width + x;
					//if( pFloorAttr[idxc].id > 0 )
					//	pFloorAttr[idxc].id *= -1;
					if( y > 0 ){
						// upper left
						if( x > 0 ){
							idx = idxc - pDepth->floor_attr_width - 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								idx_polar2 = pFloorAttr[idx].polar_id - 1;
								if( (pFloorAttr[idx].polar_id > 0) &&
									(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
									){
									pPolarQueue[idx_polar].queue[ntail].x = x - 1;
									pPolarQueue[idx_polar].queue[ntail].y = y - 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									pPolarTargets[idx_polar2].mass --;
									ntail = (ntail + 1)%queue_len;
								}
							}
						}
						// upper
						idx = idxc - pDepth->floor_attr_width;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							idx_polar2 = pFloorAttr[idx].polar_id - 1;
							if( (pFloorAttr[idx].polar_id == 0) ||
								(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
								){
								pPolarQueue[idx_polar].queue[ntail].x = x;
								pPolarQueue[idx_polar].queue[ntail].y = y - 1;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								pPolarTargets[idx_polar2].mass --;
								ntail = (ntail + 1)%queue_len;
							}
						}
						// upper right
						if( x < pDepth->floor_attr_width - 1 ){
							idx = idxc - pDepth->floor_attr_width + 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								idx_polar2 = pFloorAttr[idx].polar_id - 1;
								if( (pFloorAttr[idx].polar_id == 0) || 
									(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
									){
									pPolarQueue[idx_polar].queue[ntail].x = x + 1;
									pPolarQueue[idx_polar].queue[ntail].y = y - 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									pPolarTargets[idx_polar2].mass --;
									ntail = (ntail + 1)%queue_len;
								}
							}
						}
					}
					// left
					if( x > 0 ){
						idx = idxc - 1;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							idx_polar2 = pFloorAttr[idx].polar_id - 1;
							if( (pFloorAttr[idx].polar_id == 0) ||
								(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
								){
								pPolarQueue[idx_polar].queue[ntail].x = x - 1;
								pPolarQueue[idx_polar].queue[ntail].y = y;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								pPolarTargets[idx_polar2].mass --;
								ntail = (ntail + 1)%queue_len;
							}
						}
					}
					// right
					if( x < pDepth->floor_attr_width - 1 ){
						idx = idxc + 1;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							idx_polar2 = pFloorAttr[idx].polar_id - 1;
							if( (pFloorAttr[idx].polar_id == 0) ||
								(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
								){
								pPolarQueue[idx_polar].queue[ntail].x = x + 1;
								pPolarQueue[idx_polar].queue[ntail].y = y;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								pPolarTargets[idx_polar2].mass --;
								ntail = (ntail + 1)%queue_len;
							}
						}
					}
					if( y < pDepth->floor_attr_height - 1 ){
						// lower left
						if( x > 0 ){
							idx = idxc + pDepth->floor_attr_width - 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								idx_polar2 = pFloorAttr[idx].polar_id - 1;
								if( (pFloorAttr[idx].polar_id == 0) ||
									(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
									){
									pPolarQueue[idx_polar].queue[ntail].x = x - 1;
									pPolarQueue[idx_polar].queue[ntail].y = y + 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									pPolarTargets[idx_polar2].mass --;
									ntail = (ntail + 1)%queue_len;
								}
							}
						}
						// lower
						idx = idxc + pDepth->floor_attr_width;
						if( pFloorAttr[idx].id == root_id ){
							new_dis = pFloorAttr[idxc].polar_dis + 1;
							idx_polar2 = pFloorAttr[idx].polar_id - 1;
							if( (pFloorAttr[idx].polar_id == 0) ||
								(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
								){
								pPolarQueue[idx_polar].queue[ntail].x = x;
								pPolarQueue[idx_polar].queue[ntail].y = y + 1;
								pFloorAttr[idx].polar_id = idx_polar + 1;
								pFloorAttr[idx].polar_dis= new_dis;
								pPolarTargets[idx_polar].mass ++;
								pPolarTargets[idx_polar2].mass --;
								ntail = (ntail + 1)%queue_len;
							}
						}
						// lower right
						if( x < pDepth->floor_attr_width - 1 ){
							idx = idxc + pDepth->floor_attr_width + 1;
							if( pFloorAttr[idx].id == root_id ){
								new_dis = pFloorAttr[idxc].polar_dis + 1;
								idx_polar2 = pFloorAttr[idx].polar_id - 1;
								if( (pFloorAttr[idx].polar_id == 0) ||
									(pPolarTargets[idx_polar].mass*pFloorAttr[idx].polar_dis > pPolarTargets[idx_polar2].mass*new_dis)
									){
									pPolarQueue[idx_polar].queue[ntail].x = x + 1;
									pPolarQueue[idx_polar].queue[ntail].y = y + 1;
									pFloorAttr[idx].polar_id = idx_polar + 1;
									pFloorAttr[idx].polar_dis= new_dis;
									pPolarTargets[idx_polar].mass ++;
									pPolarTargets[idx_polar2].mass --;
									ntail = (ntail + 1)%queue_len;
								}
							}
						}
					}
				}
				pPolarQueue[idx_polar].head = k;
				pPolarQueue[idx_polar].tail = ntail;
				idx_polar ++;
			}
		}
		//sprintf(path,"b:\\floor_middle_%d.txt",iter_cnt);
		//SaveID( path, pDepth->pFloorAttr, pDepth->map, pDepth->floor_attr_width, pDepth->floor_attr_height );
		iter_cnt ++;
	}while(id_change);
	//*/

	// collect the polar targets--------------------------------------------------------------------------------
	for( i=0; i<pDepth->polar_targets_cnt; i++ ){
		pPolarTargets[i].mass = 0;
		pPolarTargets[i].floor_mass = 0;
	}
	pFloorAttr = pDepth->pFloorAttr;
	for( y=0; y<pDepth->floor_attr_height; y++ ){
		for( x=0; x<pDepth->floor_attr_width; x++ ){
			if (pFloorAttr[x].polar_id > 0){
				i = pFloorAttr[x].polar_id - 1;
				assert(i>=0);
				if( pPolarTargets[i].max_height < pFloorAttr[x].max_height ){
					pPolarTargets[i].max_height = pFloorAttr[x].max_height;
					pPolarTargets[i].x = x;
					pPolarTargets[i].y = y;
				}
				if( pPolarTargets[i].mass == 0 ){
					pPolarTargets[i].rect.left = pPolarTargets[i].rect.right = x;
					pPolarTargets[i].rect.top  = pPolarTargets[i].rect.bottom= y;
				}
				else{
					if( pPolarTargets[i].rect.left > x )
						pPolarTargets[i].rect.left = x;
					if( pPolarTargets[i].rect.right < x + 1 )
						pPolarTargets[i].rect.right = x + 1;
					if( pPolarTargets[i].rect.top > y )
						pPolarTargets[i].rect.top = y;
					if( pPolarTargets[i].rect.bottom < y + 1 )
						pPolarTargets[i].rect.bottom = y + 1;
				}
				if( pFloorAttr[x].max_height < (pDepth->floor_hthresh << 1) )
					pPolarTargets[i].floor_mass ++;
				//dis_up    = abs( pFloorAttr[x].max_v - pDep->edge_up[pFloorAttr[x].max_u] );
				//dis_left  = abs( pFloorAttr[x].max_u - pDep->edge_left[pFloorAttr[x].max_v] );
				//dis_right = abs( pFloorAttr[x].max_u - pDep->edge_right[pFloorAttr[x].max_v] );
				//if( ((pFloorAttr[x].max_height / pFloorAttr[x].prj_cnt < 150)
				//	&& ((dis_up < 10) || (dis_left < 10) || (dis_right < 10)))
				//	|| ( (pFloorAttr[x].max_height>1800) && (pFloorAttr[x].prj_cnt>50) )
				//	)
				//	pTargets[i].wall_mass += 1;//pFloorAttr[x].prj_cnt;
				pPolarTargets[i].mass += 1;//pFloorAttr[x].prj_cnt;
			}
		}
		pFloorAttr += pDepth->floor_attr_width;
	}

	// init proper value for polar targets-------------------------
	for( i=0; i<pDepth->polar_targets_cnt; i++ ){
		pPolarTargets[i].img_mass = 0;
		pPolarTargets[i].support_mass = 0;
		pPolarTargets[i].rect_world.left  = pPolarTargets[i].rect.left  * pDepth->floor_unit - (pDepth->floor_x_span>>1);
		pPolarTargets[i].rect_world.right = pPolarTargets[i].rect.right  * pDepth->floor_unit - (pDepth->floor_x_span>>1);
		pPolarTargets[i].rect_world.bottom= (pDepth->floor_attr_height - 1 - pPolarTargets[i].rect.top)   * pDepth->floor_unit + pDepth->floor_y_base;
		pPolarTargets[i].rect_world.top   = (pDepth->floor_attr_height - 1 - pPolarTargets[i].rect.bottom)* pDepth->floor_unit + pDepth->floor_y_base;
	}
	pFloorAttr = pDepth->pFloorAttr;
	pImgAttr   = pDepth->pImgAttr;
	pImgAttrT  = pDepth->pImgAttr + pDepth->width;
	for( y=0; y<pDepth->height-1; y++ ){
		for( x=0; x<pDepth->width; x++ ){
			// collect the image rect for the polar targets-----------------------
			idx = pFloorAttr[pImgAttr->offset].polar_id - 1;
			if( pFloorAttr[pImgAttr->offset].polar_id > 0 ){
				if( pPolarTargets[idx].img_mass == 0 ){
					pPolarTargets[idx].rect_image.left = pPolarTargets[idx].rect_image.right = x;
					pPolarTargets[idx].rect_image.top  = pPolarTargets[idx].rect_image.bottom= y;
				}
				else{
					if( pPolarTargets[idx].rect_image.left > x )
						pPolarTargets[idx].rect_image.left = x;
					if( pPolarTargets[idx].rect_image.right < x + 1 )
						pPolarTargets[idx].rect_image.right = x + 1;
					if( pPolarTargets[idx].rect_image.top > y )
						pPolarTargets[idx].rect_image.top = y;
					if( pPolarTargets[idx].rect_image.bottom < y + 1 )
						pPolarTargets[idx].rect_image.bottom = y + 1;
				}
				pPolarTargets[idx].img_mass ++;
			}
			pImgAttr ++;
		}
	}

}

void SaveID( char * path,  FloorAttr * pAttr, short * map, int width, int height )
{
	int y,x,k;
	FILE * pfile = fopen(path,"w");
	for( y=0; y<height; y++ ){
		for( x=0; x<width; x++ ){
			k = map[pAttr[x].id];
			//fprintf(pfile,"%5d",k);
			fprintf(pfile,"%3d%2d%3d|",pAttr[x].id,pAttr[x].polar_id,pAttr[x].polar_dis);
		}
		fprintf(pfile,"\n");
		pAttr += width;
	}
	fclose(pfile);
}

void SaveIDImg( char * path, ImgAttr * pAttr, short * map, int width, int height )
{
	int y,x,is_fg;
	FILE * pfile = fopen(path,"w");
	for( y=0; y<height; y++ ){
		for( x=0; x<width; x++ ){
			is_fg = pAttr[x].type&PT_TYPE_FG ? 1:0;
			fprintf(pfile,"%5d,%d",pAttr[x].id,is_fg);
		}
		fprintf(pfile,"\n");
		pAttr += width;
	}
	fclose(pfile);
}

void ReverseMap( Depth * pDep )
{
	ImgAttr * pImgAttr = pDep->pImgAttr;
	ImgAttr * pImgAttrE= pDep->pImgAttr + pDep->frame_len;
	FloorAttr * pFloorAttr = pDep->pFloorAttr;
	int tmp_id;

	// scan the image------------------------------------------------
	while( pImgAttr < pImgAttrE ){
		if( pImgAttr->offset >= 0 ){
			// update the rid stuff----------------------------------------------------
			tmp_id = pDep->map[pFloorAttr[pImgAttr->offset].id];
			if( (tmp_id > 0) && ((pImgAttr->height < 400)||(pImgAttr->bg_height < 300)||(abs( pImgAttr->depth - pImgAttr->avg_bg_dep) < pDep->pt_dep_dif_th)) )
				pImgAttr->active_cnt = pDep->active_cnt_max;

			if( pImgAttr->type & TARGET_TYPE_FLOOR ){
				pImgAttr->rid = 0;
			}
			else{
				pImgAttr->rid = tmp_id;
				pImgAttr->polar_id = pFloorAttr[pImgAttr->offset].polar_id;
			}
		}
		else
			pImgAttr->rid = 0;
		pImgAttr ++;
	}
	// 调试用代码----------------------------------------------------------
	for( tmp_id = 0; tmp_id < pDep->floor_targets_cnt; tmp_id ++ ){
		pDep->pTargets[tmp_id].id = pDep->map[pDep->pTargets[tmp_id].id];
	}
}

bool RefineTargets( Depth * pDep, short * map )
{
	ImgAttr * pImgAttr = pDep->pImgAttr, * pImgNeighbor;
	ImgAttr * pImgAttrE= pDep->pImgAttr + pDep->frame_len;
	ImgAttr * pImgAttrT;
	int x, y, i, j, k, nextId,m,dis;
	int vw=pDep->width;
	int vh=pDep->height;
	int nPixels=vw*vh;
	int idOfs[4],ids[4];
	RefineTarget * pImgTargets = pDep->pImgTargets;
	RefineTarget * pRTargets= pDep->pRefineTargets;
	int is_fg;
	int abh_th = pDep->floor_hthresh;
	int max_rid, max_rid_cnt,rid;
	Rect rct;
	int max_idx, max_wdis, w_dis;

	idOfs[0] = -vw-1;
	idOfs[1] = -vw;
	idOfs[2] = -vw+1;
	idOfs[3] = -1;

	// to start, every id value maps to itself
	nextId = 1;
	for( i = 0; i<=pDep->max_blobs; i++)
		map[i] = i;
	
	// scan first pixel as a special case
	is_fg = (!(pImgAttr->type & PT_TYPE_FG2BG))&&(pImgAttr->type & PT_TYPE_FG);
	if ( is_fg ){
		pImgAttr->id = nextId++;
	}
	else
		pImgAttr->id = 0;
	pImgAttr ++;
	
	// scan rest of first row as a special case
	for(x=1; x<vw; x++){
		is_fg = (!(pImgAttr->type & PT_TYPE_FG2BG))&&(pImgAttr->type & PT_TYPE_FG);
		if ( is_fg ){
			pImgNeighbor = pImgAttr - 1;
			if( (pImgAttr->type != PT_TYPE_INVALID) && (pImgNeighbor->type != PT_TYPE_INVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			j = 0;
			if( dis < pDep->pt_neighbor_th )
				j = map[pImgNeighbor->id];
			if (j > 0){
				pImgAttr->id = j;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr ++;
	}
	
	// scan rest of rows
	for(y=1; y<vh; y++){
		is_fg = (!(pImgAttr->type & PT_TYPE_FG2BG))&&(pImgAttr->type & PT_TYPE_FG);
		// check first pixel of row as a special case
		if ( is_fg ){
			m = 0;
			pImgNeighbor = pImgAttr - vw;
			if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			i = map[pImgNeighbor->id];
			if( (dis < pDep->pt_neighbor_th) && (i > m) )
				m = i;

			pImgNeighbor = pImgAttr - vw + 1;
			if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			i = map[pImgNeighbor->id];
			if( (dis < pDep->pt_neighbor_th) && (i > m) )
				m = i;
		
			if (m>0){
				pImgAttr->id = m;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr ++;
	
	    // now check the 'middle' of the row
		for(x=1; x<vw-1; x++){
			is_fg = (!(pImgAttr->type & PT_TYPE_FG2BG))&&(pImgAttr->type & PT_TYPE_FG);
			if ( is_fg ){
				j = 0;
				// find the max neighbor
				for( i = 0; i<4; i++){
					pImgNeighbor = pImgAttr + idOfs[i];
					if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
						dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
					else
						dis = 0;
					k = pImgNeighbor->id;
					if( dis < pDep->pt_neighbor_th ){
						ids[i] = map[k];
						if (ids[i] > j) 
							j = ids[i];
					}
					else{
						ids[i] = 0;
					}
				}
				if (j > 0){
					for( i = 0; i<4; i++){
						if (ids[i]==0 || ids[i]==j) 
							continue;
						for(k=1; k<nextId; k++){
							if (map[k]==ids[i])
								map[k] = j;
						}
					}
					pImgAttr->id = j;
				}
				else{
					pImgAttr->id = nextId++;
				}
			}
			else
				pImgAttr->id = 0;
			pImgAttr ++;
		}
	
		// finally, we can check the last pixel of the row as a special case
		is_fg = (!(pImgAttr->type & PT_TYPE_FG2BG))&&(pImgAttr->type & PT_TYPE_FG);
		if ( is_fg ){
			m = 0;
			pImgNeighbor = pImgAttr - vw - 1;
			if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			i = map[pImgNeighbor->id];
			if( (dis < pDep->pt_neighbor_th) && (i > m) )
				m = i;

			pImgNeighbor = pImgAttr - vw;
			if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			i = map[pImgNeighbor->id];
			if( (dis < pDep->pt_neighbor_th) && (i > m) )
				m = i;

			pImgNeighbor = pImgAttr - 1;
			if( (pImgAttr->type & PT_TYPE_DVALID) && (pImgNeighbor->type & PT_TYPE_DVALID) )
				dis = abs(pImgAttr->x - pImgNeighbor->x) + abs(pImgAttr->y - pImgNeighbor->y);
			else
				dis = 0;
			i = map[pImgNeighbor->id];
			if( (dis < pDep->pt_neighbor_th) && (i > m) )
				m = i;
			
			if (m>0){
				pImgAttr->id = m;
			}
			else{
				pImgAttr->id = nextId++;
			}
		}
		else
			pImgAttr->id = 0;
		pImgAttr++;
		
		if (nextId > pDep->max_blobs){
			printf("nextId exceeded!\n");
			return false;
		}
	}

	//SaveIDImg( pDep->pImgAttr, map, pDep->width, pDep->height );

	// collect the refined targets----------------------------------------------------------
	for( i=0; i<nextId; i++ ){
		pImgTargets[i].active_mass = 0;
		pImgTargets[i].dep_mass = 0;
		pImgTargets[i].grad_mass = 0;
		pImgTargets[i].mass = 0;
		pImgTargets[i].max_height = 0;
		pImgTargets[i].support_mass = 0;
		pImgTargets[i].floor_mass = 0;
		pImgTargets[i].prj_mass = 0;
		memset( pImgTargets[i].low_pt, 0, pDep->width );
		memset( pImgTargets[i].rid_cnt, 0, sizeof(int)*pDep->max_blobs );
	}
	pImgAttr = pDep->pImgAttr;
	for( y=0; y<pDep->height-1; y++ ){
		pImgAttrT = pImgAttr + pDep->width;
		for( x=0; x<pDep->width; x++ ){
			if (pImgAttr[x].id > 0){
				i = map[pImgAttr[x].id];
				pImgAttr[x].id = i;
				i -= 1;
				if( pImgTargets[i].max_height < pImgAttr[x].height ){
					pImgTargets[i].max_height = pImgAttr[x].height;
					pImgTargets[i].x = pImgAttr[x].x;
					pImgTargets[i].y = pImgAttr[x].y;
				}
				if( pImgTargets[i].mass == 0 ){
					pImgTargets[i].rect.left = pImgTargets[i].rect.right = x;
					pImgTargets[i].rect.top  = pImgTargets[i].rect.bottom= y;

					pImgTargets[i].rect_world.left = pImgTargets[i].rect_world.right = pImgAttr[x].x;
					pImgTargets[i].rect_world.top  = pImgTargets[i].rect_world.bottom= pImgAttr[x].y;
					pImgTargets[i].x = pImgAttr[x].x;
					pImgTargets[i].y = pImgAttr[x].y;
				}
				else{
					if( pImgTargets[i].rect.left > x )
						pImgTargets[i].rect.left = x;
					if( pImgTargets[i].rect.right < x + 1 )
						pImgTargets[i].rect.right = x + 1;
					if( pImgTargets[i].rect.top > y )
						pImgTargets[i].rect.top = y;
					if( pImgTargets[i].rect.bottom < y + 1 )
						pImgTargets[i].rect.bottom = y + 1;

					if( pImgTargets[i].rect_world.left > pImgAttr[x].x )
						pImgTargets[i].rect_world.left = pImgAttr[x].x;
					if( pImgTargets[i].rect_world.right < pImgAttr[x].x )
						pImgTargets[i].rect_world.right = pImgAttr[x].x;
					if( pImgTargets[i].rect_world.top > pImgAttr[x].y )
						pImgTargets[i].rect_world.top = pImgAttr[x].y;
					if( pImgTargets[i].rect_world.bottom < pImgAttr[x].y )
						pImgTargets[i].rect_world.bottom = pImgAttr[x].y;
				}
				pImgTargets[i].mass++;
				rid = pImgAttr[x].rid-1;
				if( rid >= 0 )
					pImgTargets[i].rid_cnt[pImgAttr[x].rid-1] ++;

				if( pImgAttr[x].active_cnt > 0 )
					pImgTargets[i].active_mass ++;
				if( pImgAttr[x].type & PT_TYPE_FG_DEP )
					pImgTargets[i].dep_mass ++;
				if( pImgAttr[x].type & PT_TYPE_FG_GRAD )
					pImgTargets[i].grad_mass ++;

				if( ( (pImgAttr[x].type & PT_TYPE_FG_DEP) ) &&
					(
					( (pImgAttrT[x].type & PT_TYPE_FLOOR) && (pImgAttr[x].height < pDep->floor_hthresh + 100) )
					|| (pImgAttrT[x].type == PT_TYPE_INVALID)
					|| ( (map[pImgAttrT[x].id] != pImgAttr[x].id) && (pImgAttrT[x].id > 0) )
					)
					){
						pImgAttr[x].type |= PT_TYPE_SUPPORT;
						if( !pImgTargets[i].low_pt[x] ){
							pImgTargets[i].low_pt[x] = 1;
							pImgTargets[i].support_mass ++;
						}
						if( pImgAttr[x].polar_id > 0 ){
							pDep->pPolarTargets[pImgAttr[x].polar_id-1].support_mass ++;
						}
				}
				else{
					if( pImgTargets[i].low_pt[x] ){
						pImgTargets[i].low_pt[x] = 0;
						pImgTargets[i].support_mass --;
					}
				}
			}
		}
		pImgAttr = pImgAttrT;
	}

	// collect the targets--------------------------------------------------------------
	pDep->refine_targets_cnt = pDep->floor_targets_cnt;
	is_fg = pDep->floor_targets_cnt+nextId;
	if( is_fg > pDep->max_blobs )
		is_fg = pDep->max_blobs;
	memset( pRTargets, 0, sizeof(RefineTarget)*pDep->max_blobs );
	for( i=0; i<pDep->floor_targets_cnt; i++ ){
		pRTargets[i].rect.left = pDep->width;
		pRTargets[i].rect.top  = pDep->height;

		// 从floor_target传递目标的物理位置和平面点数-----------------------------------------------------------------------
		rct.left =  pDep->pTargets[i].rect.left  * pDep->floor_unit - (pDep->floor_x_span>>1);
		rct.right=  pDep->pTargets[i].rect.right * pDep->floor_unit - (pDep->floor_x_span>>1);
		rct.bottom=  (pDep->floor_attr_height - 1 - pDep->pTargets[i].rect.top)   * pDep->floor_unit + pDep->floor_y_base;
		rct.top  = (pDep->floor_attr_height - 1 - pDep->pTargets[i].rect.bottom)* pDep->floor_unit + pDep->floor_y_base;
		pRTargets[i].rect_world = rct;
		pRTargets[i].floor_mass   = pDep->pTargets[i].floor_mass;
		pRTargets[i].prj_mass     = pDep->pTargets[i].mass;
		pRTargets[i].children_cnt = pDep->pTargets[i].children_cnt;
	}
	for( ; i<is_fg; i++ ){
		pRTargets[i].rect.left = pDep->width;
		pRTargets[i].rect.top  = pDep->height;
		pRTargets[i].rect_world.left = 100000;
		pRTargets[i].rect_world.top  = 100000;
	}
	for( i=0; i<nextId; i++ ){
		if( (pImgTargets[i].mass > pDep->min_blob_size)
			&&( (pImgTargets[i].dep_mass>0) || (pImgTargets[i].active_mass>0) )
		){
			max_rid=-1,max_rid_cnt=0;
			for( j=0; j<pDep->floor_targets_cnt; j++ ){
				if( pImgTargets[i].rid_cnt[j] > max_rid_cnt ){
					max_rid = j;
					max_rid_cnt = pImgTargets[i].rid_cnt[j];
				}
			}
			if( (max_rid_cnt << 7) > (pImgTargets[i].mass * pDep->ref_rid_th) ){   // 直接以RID方式合并
				pRTargets[max_rid].id = max_rid + 1;
				//if( (pImgTargets[i].rect.left == pDep->width) || ((rct.right - rct.left > 0) && (rct.bottom - rct.top > -pDep->ref_rid_merge_th)) ){ // 整体合并，按照或的方式
				{
					if( pRTargets[max_rid].max_height < pImgTargets[i].max_height ){
						pRTargets[max_rid].max_height = pImgTargets[i].max_height;
						pRTargets[max_rid].x = pImgTargets[i].x;
						pRTargets[max_rid].y = pImgTargets[i].y;
					}
					pRTargets[max_rid].rect = MergeRect( pRTargets[max_rid].rect, pImgTargets[i].rect );
					pRTargets[max_rid].mass         += pImgTargets[i].mass;
					pRTargets[max_rid].active_mass  += pImgTargets[i].active_mass;
					pRTargets[max_rid].dep_mass     += pImgTargets[i].dep_mass;
					pRTargets[max_rid].grad_mass    += pImgTargets[i].grad_mass;
					pRTargets[max_rid].support_mass += pImgTargets[i].support_mass;
				}
			}
			else{   // 列在floor_targets后面
				pRTargets[pDep->refine_targets_cnt].id = i + 1;
				pRTargets[pDep->refine_targets_cnt].max_height = pImgTargets[i].max_height;
				pRTargets[pDep->refine_targets_cnt].x = pImgTargets[i].x;
				pRTargets[pDep->refine_targets_cnt].y = pImgTargets[i].y;
				pRTargets[pDep->refine_targets_cnt].rect = pImgTargets[i].rect;
				pRTargets[pDep->refine_targets_cnt].rect_world  = pImgTargets[i].rect_world;
				pRTargets[pDep->refine_targets_cnt].mass        = pImgTargets[i].mass;
				pRTargets[pDep->refine_targets_cnt].active_mass = pImgTargets[i].active_mass;
				pRTargets[pDep->refine_targets_cnt].dep_mass    = pImgTargets[i].dep_mass;
				pRTargets[pDep->refine_targets_cnt].grad_mass   = pImgTargets[i].grad_mass;
				pRTargets[pDep->refine_targets_cnt].support_mass= pImgTargets[i].support_mass;
				pRTargets[pDep->refine_targets_cnt].floor_mass  = 0;
				pRTargets[pDep->refine_targets_cnt].prj_mass    = 0;
				pDep->refine_targets_cnt ++;
			}
		}
	}

	// 查看由光学补检的物体能否合并到平面图检出的物体上------------------------------------------------------------------------
	nextId = pDep->floor_targets_cnt;
	for( i=pDep->floor_targets_cnt; i<pDep->refine_targets_cnt; i++ ){
		if( pRTargets[i].active_mass>0 ){
			// 检测是否能够以重叠方式合并----------------------------------------------------------------------
			max_idx = -1, max_wdis = 0;
			for( j=0; j<nextId; j++ ){
				if( pRTargets[j].mass > 0 ){
					w_dis = DepthInnerDif( pRTargets[j].rect, pRTargets[i].rect );
					if( w_dis > max_wdis ){
						max_idx = j;
						max_wdis = w_dis;
					}
				}
			}
			if( max_idx >= 0 ){    // 小物体合并到大物体的合并
				if( pRTargets[max_idx].max_height < pRTargets[i].max_height ){
					pRTargets[max_idx].max_height = pRTargets[i].max_height;
					pRTargets[max_idx].x = pRTargets[i].x;
					pRTargets[max_idx].y = pRTargets[i].y;
				}
				pRTargets[max_idx].rect = MergeRect( pRTargets[max_idx].rect, pRTargets[i].rect );
				if( max_idx >= pDep->floor_targets_cnt )
					pRTargets[max_idx].rect_world = MergeRect( pRTargets[max_idx].rect_world, pRTargets[i].rect_world );
				pRTargets[max_idx].mass        += pRTargets[i].mass;
				pRTargets[max_idx].active_mass += pRTargets[i].active_mass;
				pRTargets[max_idx].dep_mass    += pRTargets[i].dep_mass;
				pRTargets[max_idx].grad_mass   += pRTargets[i].grad_mass;
				pRTargets[max_idx].support_mass+= pRTargets[i].support_mass;
				pRTargets[max_idx].floor_mass  += pRTargets[i].floor_mass;
				pRTargets[max_idx].prj_mass    += pRTargets[i].prj_mass;
			}
			else{
				pRTargets[nextId].id = pRTargets[i].id;
				pRTargets[nextId].max_height = pRTargets[i].max_height;
				pRTargets[nextId].x = pRTargets[i].x;
				pRTargets[nextId].y = pRTargets[i].y;
				pRTargets[nextId].rect = pRTargets[i].rect;
				pRTargets[nextId].rect_world  = pRTargets[i].rect_world;
				pRTargets[nextId].mass        = pRTargets[i].mass;
				pRTargets[nextId].active_mass = pRTargets[i].active_mass;
				pRTargets[nextId].dep_mass    = pRTargets[i].dep_mass;
				pRTargets[nextId].grad_mass   = pRTargets[i].grad_mass;
				pRTargets[nextId].support_mass= pRTargets[i].support_mass;
				pRTargets[nextId].floor_mass  = pRTargets[i].floor_mass;
				pRTargets[nextId].prj_mass    = pRTargets[i].prj_mass;
				nextId ++;
			}
		}
	}
	pDep->refine_targets_cnt = nextId;
	//pDep->refine_targets_cnt = pDep->floor_targets_cnt;// ignore all the rgb targets

	// 由Refine的目标更新polar目标的相关信息------------------------------------------
	j = 0;
	for( i=0; i<pDep->floor_targets_cnt; i++ ){
		if( pRTargets[i].children_cnt == 1 ){
			pDep->pPolarTargets[j].mass         = pRTargets[i].mass;
			pDep->pPolarTargets[j].img_mass     = pRTargets[i].mass;
			pDep->pPolarTargets[j].floor_mass   = pRTargets[i].floor_mass;
			pDep->pPolarTargets[j].support_mass = pRTargets[i].support_mass;
			pDep->pPolarTargets[j].prj_mass     = pRTargets[i].prj_mass;
			pDep->pPolarTargets[j].dep_mass     = pRTargets[i].dep_mass;
			pDep->pPolarTargets[j].prj_mass     = pRTargets[i].prj_mass;
			pDep->pPolarTargets[j].max_height   = pRTargets[i].max_height;
			pDep->pPolarTargets[j].x            = pRTargets[i].x;
			pDep->pPolarTargets[j].y            = pRTargets[i].y;
			pDep->pPolarTargets[j].rect_world   = pRTargets[i].rect_world;
			pDep->pPolarTargets[j].rect_image   = pRTargets[i].rect;
			j++;
		}
		else{
			for( k=0; k<pRTargets[i].children_cnt; k++ ){
				pDep->pPolarTargets[j].mass     = pDep->pPolarTargets[j].img_mass;
				pDep->pPolarTargets[j].dep_mass = pDep->pPolarTargets[j].img_mass;
				pDep->pPolarTargets[j].prj_mass = pDep->pPolarTargets[j].img_mass;
				j++;
			}
		}
	}
	// 为纯RGB目标构造polar对象------------------------------------------
	j = pDep->polar_targets_cnt;
	for( i=pDep->floor_targets_cnt; i<pDep->refine_targets_cnt; i++ ){
		pDep->pPolarTargets[j].mass       = pDep->pRefineTargets[i].mass;
		pDep->pPolarTargets[j].img_mass     = pRTargets[i].mass;
		pDep->pPolarTargets[j].floor_mass   = pRTargets[i].floor_mass;
		pDep->pPolarTargets[j].support_mass = pRTargets[i].support_mass;
		pDep->pPolarTargets[j].prj_mass     = pRTargets[i].prj_mass;
		pDep->pPolarTargets[j].dep_mass     = pRTargets[i].dep_mass;
		pDep->pPolarTargets[j].prj_mass     = pRTargets[i].prj_mass;
		pDep->pPolarTargets[j].max_height   = pRTargets[i].max_height;
		pDep->pPolarTargets[j].x            = pRTargets[i].x;
		pDep->pPolarTargets[j].y            = pRTargets[i].y;
		pDep->pPolarTargets[j].rect_world   = pRTargets[i].rect_world;
		pDep->pPolarTargets[j].rect_image   = pRTargets[i].rect;
		pDep->pPolarTargets[j].avg_height   = -1;
		pDep->pPolarTargets[j].root         = i;
		pRTargets[i].children_cnt = 1;
		j++;
	}
	pDep->polar_targets_cnt = j;

	// 最终收集数据，删除没有被任何ImgTargets命中的目标----------------------
	j = 0;       // index the pRTargets
	nextId = 0;  // index the pPolarTargets
	m = 0;
	for( i=0; i<pDep->refine_targets_cnt; i++ ){
		if( pRTargets[i].rect.left < pRTargets[i].rect.right ){
			pRTargets[j] = pRTargets[i];
			for( k=0; k<pRTargets[i].children_cnt; k++ ){
				pDep->pPolarTargets[nextId] = pDep->pPolarTargets[m+k];
				pDep->pPolarTargets[nextId].root = j;
				nextId ++;
			}
			j++;
		}
		m += pRTargets[i].children_cnt;
	}
	pDep->refine_targets_cnt = j;
	pDep->polar_targets_cnt  = nextId;
	return true;
}

void TrackTargets( Depth * pDep )
{
	DepObject    * pFinalObjs = pDep->pFinalObjs;
	DepObject    * pObjs = pDep->pObjs;
	RefineTarget * pRTargets = pDep->pRefineTargets;
	PolarTarget  * pPolar = pDep->pPolarTargets;
	int i,j,k,dif,dif2,wdif,wdif2,hdif,idx;
	Rect predict;
	int inner_idx, inner_val;
	int new_obj_cnt;
	int people_cnt;
	int obj_x,obj_y;
	int area,min_dif,is_valid;
	int follow_people_cnt;
	int fight_people_cnt;
	Point s,e,line_s,line_e,line_dir;
	int cur_pos_idx, last_pos_idx;
	time_t cur_tm;
	clock_t cur_clock;
	float speed, tspeed, mov_x, mov_y;
	int   idx0, idx1;

#ifdef _DEBUG
	cur_tm = pDep->frame_cnt/30;
	cur_clock = pDep->frame_cnt*CLOCKS_PER_SEC/30;
#else
	cur_tm = time(NULL);
	cur_clock = clock();
#endif
	pDep->cur_tm = cur_tm;

	// track the polar targets-----------------------------------------------------------
	for( i=0; i<pDep->n_objs; i++ )
		pDep->pObjs[i].root = -1;
	new_obj_cnt = pDep->n_objs;
	memset( pDep->track_tag, 0, sizeof(char)*pDep->n_objs );
	for( i=0; i<pDep->polar_targets_cnt; i++ ){
		inner_idx = -1, inner_val = 0;
		for( j=0; j<pDep->n_objs; j++ ){
			if( pDep->track_tag[j] != -1 ){
				// 要想匹配成功，首先要求高度必须匹配且inner距离匹配-------------------------------
				hdif = abs( pObjs[j].height - pPolar[i].max_height );
				// 依据速度，预测本帧的新位置----------------------------------------
				predict.left = pObjs[j].region_world.left + pObjs[j].speed_x;
				predict.right = pObjs[j].region_world.right + pObjs[j].speed_x;
				predict.top = pObjs[j].region_world.top + pObjs[j].speed_y;
				predict.bottom = pObjs[j].region_world.bottom + pObjs[j].speed_y;
				// 匹配自身的Inner距离
				dif = DepthInnerDif( pObjs[j].region_world, pPolar[i].rect_world );
				dif2 = DepthInnerDif( predict, pPolar[i].rect_world );
				if( dif < dif2 )
					dif = dif2;
				// 计算Outer距离-------------------------------
				wdif= DepthOuterDif( pObjs[j].region_world, pPolar[i].rect_world );
				wdif2 = DepthOuterDif( predict, pPolar[i].rect_world );
				if( wdif < wdif2 )
					wdif = wdif2;
				// 若新目标的root有多于一个polar，则尝试匹配新目标的root，计算Outer距离（处理大变小）
				if( pRTargets[pPolar[i].root].children_cnt > 1 ){
					// 匹配root的Outer距离
					dif = DepthOuterDif( pObjs[j].region_world, pRTargets[pPolar[i].root].rect_world );
					dif2 = DepthOuterDif( predict, pRTargets[pPolar[i].root].rect_world );
					if( dif < dif2 )
						dif = dif2;
					if( wdif < dif )
						wdif = dif;
				}
				wdif2 = 0;
				if( (pPolar[i].avg_height < 0) && (pObjs[j].type == TARGET_TYPE_FLOOR) ){
					wdif2 = DepthOuterDif( pObjs[i].region, pObjs[j].region );
				}
				if( (true
					&& ((hdif<<7)<pDep->track_height_th*pObjs[j].max_height)
					&& ((dif > pDep->track_inner_th) || (wdif > pDep->track_xy_th)))
					|| ( wdif2 > pDep->track_uv_th )
					){
					if( (inner_val < pObjs[j].det_cnt) ){
						inner_idx = j;
						inner_val = pObjs[j].det_cnt;
					}
				}
			}
		}

		if( inner_idx >= 0 ){ // track
			pDep->track_tag[inner_idx] = -1;

			pObjs[inner_idx].lregion = pObjs[inner_idx].region_world;
			pObjs[inner_idx].region  = pPolar[i].rect_image;
			pObjs[inner_idx].region_world = pPolar[i].rect_world;
			pObjs[inner_idx].height = pPolar[i].max_height;
			pObjs[inner_idx].region_prj_last = pObjs[inner_idx].region_prj;
			pObjs[inner_idx].region_prj = pPolar[i].rect;
			if( pObjs[inner_idx].max_height < pPolar[i].max_height ){
				pObjs[inner_idx].max_height = pPolar[i].max_height;
				pObjs[inner_idx].cwidth = pObjs[inner_idx].region_world.right - pObjs[inner_idx].region_world.left;
				pObjs[inner_idx].cheight= pObjs[inner_idx].region_world.bottom - pObjs[inner_idx].region_world.top;
			}
			if( pObjs[inner_idx].life < pDep->track_max_life )
				pObjs[inner_idx].life += 2;
			pObjs[inner_idx].det_cnt ++;
			pObjs[inner_idx].cx = pPolar[i].x;
			pObjs[inner_idx].cy = pPolar[i].y;
			pObjs[inner_idx].mass = pPolar[i].mass;
			pObjs[inner_idx].dep_mass = pPolar[i].dep_mass;
			pObjs[inner_idx].support_mass = pPolar[i].support_mass;
			pObjs[inner_idx].floor_mass = pPolar[i].floor_mass;
			pObjs[inner_idx].prj_mass   = pPolar[i].prj_mass;
		}
		else{ // new
			pObjs[new_obj_cnt].region = pPolar[i].rect_image;
			pObjs[new_obj_cnt].region_world = pPolar[i].rect_world;
			pObjs[new_obj_cnt].height = pPolar[i].max_height;
			pObjs[new_obj_cnt].region_prj = pPolar[i].rect;
			pObjs[new_obj_cnt].region_prj_last = pObjs[new_obj_cnt].region_prj;
			pObjs[new_obj_cnt].max_height = pPolar[i].max_height;
			pObjs[new_obj_cnt].cwidth = pPolar[i].rect_world.right - pPolar[i].rect_world.left;
			pObjs[new_obj_cnt].cheight = pPolar[i].rect_world.bottom - pPolar[i].rect_world.top;
			pObjs[new_obj_cnt].speed_x = 0;
			pObjs[new_obj_cnt].speed_y = 0;
			pObjs[new_obj_cnt].speed   = 0;
			pObjs[new_obj_cnt].max_move_cnt = 0;
			pObjs[new_obj_cnt].life = 1;
			pObjs[new_obj_cnt].det_cnt = 1;
			pObjs[new_obj_cnt].cx = pPolar[i].x;
			pObjs[new_obj_cnt].cy = pPolar[i].y;
			pObjs[new_obj_cnt].mass = pPolar[i].mass;
			pObjs[new_obj_cnt].dep_mass = pPolar[i].dep_mass;
			pObjs[new_obj_cnt].support_mass = pPolar[i].support_mass;
			pObjs[new_obj_cnt].floor_mass = pPolar[i].floor_mass;
			pObjs[new_obj_cnt].prj_mass  = pPolar[i].prj_mass;
			pObjs[new_obj_cnt].floor_dif = 0;
			pObjs[new_obj_cnt].root  = pPolar[i].root;
			pObjs[new_obj_cnt].in_region = 0;
			pObjs[new_obj_cnt].accu_move = 0;
			pObjs[new_obj_cnt].normal_cnt = 0;
			pObjs[new_obj_cnt].float_cnt = 0;
			pObjs[new_obj_cnt].floor_cnt = 0;
			pObjs[new_obj_cnt].people_cnt = 0;
			pObjs[new_obj_cnt].move_cnt = 0;
			pObjs[new_obj_cnt].wander_cnt = 0;
			pObjs[new_obj_cnt].fight_cnt = 0;
			pObjs[new_obj_cnt].last_fight_det_cnt = 0;
			pObjs[new_obj_cnt].owner = -1;
			pObjs[new_obj_cnt].last_owner = -1;
			pObjs[new_obj_cnt].owner_dis = 0;
			pObjs[new_obj_cnt].owner_cnt = 0;
			pObjs[new_obj_cnt].status = TARGET_STATUS_STAY_STILL;
			pObjs[new_obj_cnt].tm_still = cur_tm;
			pObjs[new_obj_cnt].is_evt = 0;
			pObjs[new_obj_cnt].is_evted = 0;
			pObjs[new_obj_cnt].tm_det = cur_tm;
			pObjs[new_obj_cnt].tm_lie = cur_tm;
			memset( pObjs[new_obj_cnt].his_cx, 0, sizeof(pObjs[new_obj_cnt].his_cx) );
			memset( pObjs[new_obj_cnt].his_cy, 0, sizeof(pObjs[new_obj_cnt].his_cy) );
			pObjs[new_obj_cnt].cur_his = 0;
			new_obj_cnt ++;
		}
	}
	pDep->n_objs = new_obj_cnt;

	// 删除生命值为0的目标，且计算目标的各项参数，初始化目标间属性--------------------------------
	new_obj_cnt = 0;
	for( i=0; i<pDep->n_objs; i++ ){
		obj_x = (pObjs[i].region_world.left + pObjs[i].region_world.right) >> 1;
		obj_y = (pObjs[i].region_world.top + pObjs[i].region_world.bottom) >> 1;

		pObjs[i].his_cx[pObjs[i].cur_his] = obj_x;
		pObjs[i].his_cy[pObjs[i].cur_his] = obj_y;
		pObjs[i].his_tm[pObjs[i].cur_his] = cur_clock;

		// 各种参数的计算-------------------------------------------------------------------------
		if( pObjs[i].det_cnt == 1 ){
			pObjs[i].ori_cx = obj_x;
			pObjs[i].ori_cy = obj_y;
			pObjs[i].speed_x= 0;
			pObjs[i].speed_y= 0;
			speed = 0;
			pObjs[i].wander_cx[pObjs[i].wander_cnt] = obj_x;
			pObjs[i].wander_cy[pObjs[i].wander_cnt] = obj_x;
			pObjs[i].wander_cnt ++;
		}
		else{
			if( pObjs[i].det_cnt < MAX_TARGET_HISTORY ){
				idx0 = (pObjs[i].cur_his - 1 + MAX_TARGET_HISTORY) % MAX_TARGET_HISTORY;
				pObjs[i].speed_x= obj_x - pObjs[i].his_cx[idx0];
				pObjs[i].speed_y= obj_y - pObjs[i].his_cy[idx0];
				speed = 0;
			}
			else{
				speed = 10000000;
				for( idx0 = 0; idx0<MAX_TARGET_HISTORY; idx0++ ){
					for( idx1 = idx0 + 1; idx1<MAX_TARGET_HISTORY; idx1++ ){
						mov_x = (float)(pObjs[i].his_cx[idx0] - pObjs[i].his_cx[idx1]);
						mov_y = (float)(pObjs[i].his_cy[idx0] - pObjs[i].his_cy[idx1]);
						tspeed =(float)sqrt( mov_x*mov_x + mov_y*mov_y );
						mov_x = (float)((double)(pObjs[i].his_tm[idx1] - pObjs[i].his_tm[idx0])/(double)CLOCKS_PER_SEC);
						if( mov_x < 0 )
							mov_x = -mov_x;
						tspeed /= mov_x;
						if( tspeed < speed )
							speed = tspeed;
					}
				}
				idx0 = (pObjs[i].cur_his - 3 + MAX_TARGET_HISTORY) % MAX_TARGET_HISTORY;
				pObjs[i].speed_x= (obj_x - pObjs[i].his_cx[idx0])/3;
				pObjs[i].speed_y= (obj_y - pObjs[i].his_cy[idx0])/3;
			}
		}
		pObjs[i].speed = (int)speed;
		pObjs[i].cur_his = (pObjs[i].cur_his + 1) % MAX_TARGET_HISTORY;

		pObjs[i].floor_dif = 0;//FloorDif( pDep, pObjs[i].region_prj, pObjs[i].region_prj_last );

		// 目标状态的计算----------------------------------------------------------------------------
		if( pObjs[i].type != TARGET_TYPE_FLOOR ){
			if( pObjs[i].speed <= pDep->track_still_th ){
				pObjs[i].is_evted &= ~EVT_RUNNING;
				if( pObjs[i].status & TARGET_STATUS_MOVE ){
					pObjs[i].status |= TARGET_STATUS_STAY_STILL;
					pObjs[i].status &= ~TARGET_STATUS_MOVE;
					pObjs[i].tm_still = cur_tm;
					// 处理wander历史
					dif = (int)sqrt( (float)( (pObjs[i].ori_cx - obj_x)*(pObjs[i].ori_cx - obj_x) + (pObjs[i].ori_cy - obj_y)*(pObjs[i].ori_cy - obj_y) ) );
					if( dif > pDep->track_wander_th ){
						pObjs[i].is_evted &= ~EVT_SAMPLE_FRAME;
						idx = pObjs[i].wander_cnt % MAX_WANDER_HISTORY;
						pObjs[i].accu_move += dif;
						pObjs[i].wander_cx[idx] = obj_x;
						pObjs[i].wander_cy[idx] = obj_y;
						pObjs[i].ori_cx = obj_x;
						pObjs[i].ori_cy = obj_y;
						pObjs[i].wander_cnt ++;
					}
				}
			}
			else{
				if( pObjs[i].status & TARGET_STATUS_STAY_STILL ){
					pObjs[i].status &= ~TARGET_STATUS_STAY_STILL;
					pObjs[i].status |= TARGET_STATUS_MOVE;
					pObjs[i].move_cnt = 0;
				}
				else
					pObjs[i].move_cnt ++;
			}
		}

		if( (pObjs[i].height << 7) < (pObjs[i].max_height * pDep->people_down_th) ){
			if(  (pObjs[i].height << 7) < (pObjs[i].max_height * pDep->people_lie_th) ) 
			{
				pObjs[i].status &= ~TARGET_STATUS_DOWN;
				if ( (pObjs[i].status&TARGET_STATUS_LIEDOWN) == 0){
					pObjs[i].tm_lie = cur_tm;
					pObjs[i].is_evted &= ~EVT_LIEDOWN_LONG;
					pObjs[i].is_evted &= ~EVT_LIEDOWN;
					pObjs[i].status |= TARGET_STATUS_LIEDOWN;
				}
			}
			else{
				pObjs[i].status &= ~TARGET_STATUS_LIEDOWN;
				if ((pObjs[i].status&TARGET_STATUS_DOWN) == 0){
					pObjs[i].tm_down = cur_tm;
					pObjs[i].is_evted &= ~EVT_DOWN;
					pObjs[i].is_evted &= ~EVT_DOWN;
					pObjs[i].status |= TARGET_STATUS_DOWN;
				}
			}
		}
		else{
			pObjs[i].status &= ~TARGET_STATUS_LIEDOWN;
			pObjs[i].status &= ~TARGET_STATUS_LIEDOWN;
		}

		// 目标类型的判断 ---------------------------
		area = (pObjs[i].region_world.right - pObjs[i].region_world.left) * (pObjs[i].region_world.bottom - pObjs[i].region_world.top);
		if( (pObjs[i].dep_mass << 7) < pObjs[i].mass * pDep->floor_pt_th ){
			pObjs[i].floor_cnt ++;
			pObjs[i].people_cnt = 0;
			pObjs[i].normal_cnt = 0;
			pObjs[i].float_cnt = 0;
		}
		else{
			k = pObjs[i].region.right - pObjs[i].region.left;
			if( (pObjs[i].support_mass << 7) >  k * pDep->ref_support_th 
				){
					// 只对有支撑的物体计算最大连续运动帧数
					if( pObjs[i].max_move_cnt < pObjs[i].move_cnt )
						pObjs[i].max_move_cnt = pObjs[i].move_cnt;
					if( (pObjs[i].dep_mass > (pObjs[i].mass>>1))
						&&(area > pDep->people_area_th)
						&&( (pObjs[i].max_move_cnt > pDep->people_veri_cnt_th) ||
						(pObjs[i].max_move_cnt>=0)&&(pObjs[i].max_height > pDep->ref_people_th) )
					)
				{
					pObjs[i].people_cnt ++;
				}
				else
					pObjs[i].normal_cnt ++;
			}
			else
				pObjs[i].float_cnt ++;
		}
		// 防止某些高度过低的物体被认作是人
		if( pObjs[i].max_height < 600 )
			pObjs[i].people_cnt = 0;

		pObjs[i].type = TARGET_TYPE_NORMAL;
		dif = pObjs[i].normal_cnt;
		if( (pObjs[i].people_cnt > dif) && (pObjs[i].det_cnt > pDep->people_veri_cnt_th) ){
			pObjs[i].type = TARGET_TYPE_PEOPLE;
			dif = pObjs[i].people_cnt;
		}
		if( pObjs[i].float_cnt > dif ){
			pObjs[i].type = TARGET_TYPE_FLOAT;
			dif = pObjs[i].float_cnt;
		}
		if( pObjs[i].floor_cnt > dif ){
			pObjs[i].type = TARGET_TYPE_FLOOR;
			dif = pObjs[i].floor_cnt;
		}

		if( pObjs[i].life >= 1 ){
			pObjs[new_obj_cnt] = pObjs[i];
			pObjs[new_obj_cnt].life --;
			pObjs[new_obj_cnt].status &= ~TARGET_STATUS_FOLLOW;
			pObjs[new_obj_cnt].last_owner = pObjs[new_obj_cnt].owner;
			pObjs[new_obj_cnt].owner  = -1;
			new_obj_cnt ++;
		}
	}
	pDep->n_objs = new_obj_cnt;

	// 计算目标间属性----------------------------------------------------------------------------------
	follow_people_cnt = 0;
	fight_people_cnt  = 0;
	for( i=0; i<pDep->n_objs; i++ ){
		min_dif = 100000;
		idx = 0;
		if( pObjs[i].type == TARGET_TYPE_PEOPLE ){
			for( j=0; j<pDep->n_objs; j++ ){
				if( i==j )
					continue;
				if( pObjs[j].type == TARGET_TYPE_PEOPLE ){
					dif = RectDistance( pObjs[i].region_world, pObjs[j].region_world );
					if( dif < min_dif ){
						min_dif = dif;
						idx = j;
					}
				}
				else{
					dif = RectDistance( pObjs[i].region_world, pObjs[j].region_world );
					if( dif < pDep->track_owner_th ){
						pObjs[j].status |= TARGET_STATUS_FOLLOW;
						if( pObjs[j].owner == -1 ){
							pObjs[j].owner = i;
							pObjs[j].owner_dis = dif;
						}
						else{
							if( pObjs[j].owner_dis > dif ){
								pObjs[j].owner = i;
								pObjs[j].owner_dis = dif;
							}
						}
					}
				}
			}
			j = idx;
			if( min_dif < pDep->track_near_th ){
				// 判断follow状态---------------------------------------------
				if( (pObjs[i].status & TARGET_STATUS_FOLLOW) == 0 ){
					pObjs[i].status |= TARGET_STATUS_FOLLOW;
					pObjs[i].owner = j;
					pObjs[i].tm_follow = cur_tm;
				}
				else{
					if( pObjs[i].owner != pObjs[i].last_owner )
						pObjs[i].tm_follow = cur_tm;
					if( cur_tm - pObjs[i].tm_follow > pDep->people_follow_th )
						follow_people_cnt ++;
				}
				// 判断是否打架---------------------------------------------------
				if( min_dif == 0 ){
					if( pObjs[i].fight_cnt == 0 ){
						pObjs[i].fight_cnt ++;
					}
					else if( pObjs[i].fight_cnt > 0 ){
						dif = pObjs[i].det_cnt - pObjs[i].last_fight_det_cnt;
						if( dif > pDep->people_fight_det_th )
							pObjs[i].fight_cnt = 0;
					}
					else{
						pObjs[i].fight_cnt = -pObjs[i].fight_cnt;
						pObjs[i].fight_cnt ++;
						if( pObjs[i].fight_cnt == pDep->people_fight_th )
							fight_people_cnt ++;
					}
					pObjs[i].last_fight_det_cnt = pObjs[i].det_cnt;
				}
				else{
					if( pObjs[i].fight_cnt > 0 ){
						pObjs[i].fight_cnt ++;
						pObjs[i].fight_cnt = - pObjs[i].fight_cnt;
						if( -pObjs[i].fight_cnt == pDep->people_fight_th )
							fight_people_cnt ++;
					}
					else if( pObjs[i].fight_cnt < 0 ){
						dif = pObjs[i].det_cnt - pObjs[i].last_fight_det_cnt;
						if( dif > pDep->people_fight_det_th )
							pObjs[i].fight_cnt = 0;
					}
				}
			}
		}
	}

	// detect the event for each target---------------------------------
	pDep->people_cnt = 0;
	for( i=0; i<pDep->n_objs; i++ ){
		// 计算呈现位置-----------------------------------------------------
		//*
		World2UV( pDep, &pObjs[i].region_floor[0].x,  &pObjs[i].region_floor[0].y, pObjs[i].region_world.left,  pObjs[i].region_world.top, 0 );
		World2UV( pDep, &pObjs[i].region_floor[1].x,  &pObjs[i].region_floor[1].y, pObjs[i].region_world.right, pObjs[i].region_world.top, 0 );
		World2UV( pDep, &pObjs[i].region_floor[2].x,  &pObjs[i].region_floor[2].y, pObjs[i].region_world.right, pObjs[i].region_world.bottom, 0 );
		World2UV( pDep, &pObjs[i].region_floor[3].x,  &pObjs[i].region_floor[3].y, pObjs[i].region_world.left,  pObjs[i].region_world.bottom, 0 );
		World2UV( pDep, &pObjs[i].region_head[0].x,   &pObjs[i].region_head[0].y,  pObjs[i].region_world.left,  pObjs[i].region_world.top, pObjs[i].height );
		World2UV( pDep, &pObjs[i].region_head[1].x,   &pObjs[i].region_head[1].y,  pObjs[i].region_world.right, pObjs[i].region_world.top, pObjs[i].height );
		World2UV( pDep, &pObjs[i].region_head[2].x,   &pObjs[i].region_head[2].y,  pObjs[i].region_world.right, pObjs[i].region_world.bottom, pObjs[i].height );
		World2UV( pDep, &pObjs[i].region_head[3].x,   &pObjs[i].region_head[3].y,  pObjs[i].region_world.left,  pObjs[i].region_world.bottom, pObjs[i].height );
		//*/

		// 依据owner对still_cnt进行调整-------------------------------------
		if( pObjs[i].type != TARGET_TYPE_PEOPLE ){
			if( (pObjs[i].owner >= 0) && (pObjs[i].owner != pObjs[i].last_owner) )  // 在last_owner=-1时也加一，这是为了在第一次有owner时，就让owner_cnt=1
				pObjs[i].owner_cnt ++;
			if( pObjs[i].owner >= 0 )   // 如果有owner，则无法判定该物体是光影还是真物体，所以总要清零still_cnt
				pObjs[i].tm_still = cur_tm;
		}
		is_valid = 1;
		if( pDep->valid_region.status != REGION_STATUS_UNSET ){
			if( IsPtInPolygon( pObjs[i].cx, pObjs[i].cy, pDep->valid_region.x, pDep->valid_region.y, pDep->valid_region.n_pt ) 
				|| IsPtInPolygon( (pObjs[i].region_world.left + pObjs[i].region_world.right)>>1, 
				                  (pObjs[i].region_world.top + pObjs[i].region_world.bottom)>>1, 
				                   pDep->valid_region.x, pDep->valid_region.y, pDep->valid_region.n_pt ) 
				){
				is_valid = 1;
				if( pObjs[i].type == TARGET_TYPE_PEOPLE )
					pDep->people_cnt ++;
			}
			else
				is_valid = 0;
		}
		pObjs[i].is_evt = 0;
		if( is_valid && (pObjs[i].type == TARGET_TYPE_PEOPLE) ){
			if( pObjs[i].status & TARGET_STATUS_LIEDOWN ){
				if( ((pObjs[i].is_evted & EVT_LIEDOWN)==0) && (cur_tm - pObjs[i].tm_lie >= pDep->people_lie_cnt_th) )
					pObjs[i].is_evt |= EVT_LIEDOWN;
				if( ((pObjs[i].is_evted & EVT_LIEDOWN_LONG)==0) && (cur_tm - pObjs[i].tm_lie >= pDep->people_lie_long_th) )
					pObjs[i].is_evt |= EVT_LIEDOWN_LONG;
			}
			if( (pObjs[i].status & TARGET_STATUS_DOWN) && ((pObjs[i].is_evted & EVT_DOWN)==0) && (cur_tm - pObjs[i].tm_down >= pDep->people_down_cnt_th) )
				pObjs[i].is_evt |= EVT_DOWN;
			if( ((pObjs[i].is_evted & EVT_STAY) == 0) && (cur_tm - pObjs[i].tm_det >= pDep->people_stay_th) )
				pObjs[i].is_evt |= EVT_STAY;
			if( (pObjs[i].accu_move >= pDep->people_wander_dis_th) && ((pObjs[i].is_evted&EVT_WANDER)==0) )
				pObjs[i].is_evt |= EVT_WANDER;
			if( ((pObjs[i].is_evted & EVT_RUNNING)==0) && (pObjs[i].speed > pDep->track_run_th) && (cur_tm-pObjs[i].tm_run>=pDep->people_run_th) ){
				pObjs[i].is_evt |= EVT_RUNNING;
				pObjs[i].tm_run = cur_tm;
			}
			if( (pObjs[i].wander_cnt>0) && ( (pObjs[i].is_evted&EVT_SAMPLE_FRAME) == 0 ) )
				pObjs[i].is_evt |= EVT_SAMPLE_FRAME;
		}
		else if(pObjs[i].type != TARGET_TYPE_PEOPLE){
			if( (cur_tm - pObjs[i].tm_still >= pDep->object_still_th) && ((pObjs[i].is_evted & (EVT_PLANE_STILL|EVT_STILL))==0) ){
				if( pObjs[i].owner < 0 ){
					if( CheckTrack( pDep, i ) == 0 ){
						if( pObjs[i].type == TARGET_TYPE_FLOOR )
							pObjs[i].is_evt |= EVT_PLANE_STILL;
						else
							pObjs[i].is_evt |= EVT_STILL;
					}
				}
			}
		}
		if( pObjs[i].is_evt )
			pObjs[i].is_evted |= pObjs[i].is_evt;
	}

	// update the info of the regions------------------------------------------------------------------------------------------
	for( i=0; i<REGION_MAX_CNT; i++ ){
		if( pDep->regions[i].status != REGION_STATUS_UNSET ){
			pDep->regions[i].is_evt = 0;
			people_cnt = 0;
			if( pDep->regions[i].type == REGION_TYPE_VIO ){
				if( pDep->regions[i].dfg_cnt > pDep->door_fg_thresh ){
					if( (pDep->regions[i].big_fg_frames <= pDep->cover_span)
						&& (pDep->people_cnt == 1)
						)
						pDep->regions[i].big_fg_frames ++;
				}
				else
					pDep->regions[i].big_fg_frames = 0;
				if( pDep->regions[i].big_fg_frames == pDep->cover_span )
					pDep->regions[i].is_evt = EVT_CAM_SINGLE_OPEN;
			}
			else if( pDep->regions[i].type == REGION_TYPE_CROSS ){
				for( j=0; j<pDep->n_objs; j++ ){
					cur_pos_idx = pObjs[j].cur_his;
					last_pos_idx= (pObjs[j].cur_his + MAX_TARGET_HISTORY - 1);
					s.x = pObjs[j].his_cx[last_pos_idx];
					s.y = pObjs[j].his_cy[last_pos_idx];
					e.x = pObjs[j].his_cx[cur_pos_idx];
					e.y = pObjs[j].his_cy[cur_pos_idx];
					line_dir.x = pDep->regions[i].x[pDep->regions[i].n_pt-1];
					line_dir.y = pDep->regions[i].y[pDep->regions[i].n_pt-1];
					for( k=0; k<pDep->regions[i].n_pt-1; k++ ){
						line_s.x = pDep->regions[i].x[k];
						line_s.y = pDep->regions[i].y[k];
						line_e.x = pDep->regions[i].x[k+1];
						line_e.y = pDep->regions[i].y[k+1];
						if( IsCross( s, e, line_s, line_e, line_dir ) ){
							pDep->regions[i].is_evt = EVT_REGION_CROSS;
							pDep->regions[i].cur_people_cnt ++;
							break;
						}
					}
				}
			}
			else{
				for( j=0; j<pDep->n_objs; j++ ){
					if( (pObjs[j].type == TARGET_TYPE_PEOPLE) && 
						( IsPtInPolygon( pObjs[j].cx, pObjs[j].cy, pDep->regions[i].x, pDep->regions[i].y, pDep->regions[i].n_pt )  
						||IsPtInPolygon( (pObjs[j].region_world.left+pObjs[j].region_world.right)>>1, 
										 (pObjs[j].region_world.top+pObjs[j].region_world.bottom)>>1, 
										 pDep->regions[i].x, pDep->regions[i].y, pDep->regions[i].n_pt )  
						)
						){
						pObjs[j].in_region |= 1<<i;
						people_cnt ++;
					}
				}
				HandleRegionPeopleCnt( pDep, pDep->regions+i, people_cnt );
			}
		}
	}

	if( (pDep->follow_people_cnt==0) && (follow_people_cnt >= 2) && (pDep->people_cnt <= 3) ){
		pDep->cam_evt = EVT_CAM_SOMEFOLLOW;
	}
	pDep->follow_people_cnt = follow_people_cnt;
	if( (pDep->fight_people_cnt==0) && (fight_people_cnt>0) ){
		pDep->cam_evt = EVT_CAM_FIGHT;
		pDep->fight_people_cnt = 100;
	}
	if( pDep->fight_people_cnt > 0 )
		pDep->fight_people_cnt --;

	// handle the valid region----------------------------------------------
	if( pDep->valid_region.type != REGION_STATUS_UNSET )
		HandleRegionPeopleCnt( pDep, &pDep->valid_region, pDep->people_cnt );

	// collect the final results---------------------------------------------
	pDep->n_final_objs = 0;
	if( pDep->people_cnt > 0 ){
		pDep->no_people_cnt = 0;
	}
	else{
		pDep->no_people_cnt ++;
	}
	for( i=0; i<pDep->n_objs; i++ ){
		if( ((pObjs[i].owner == -1)&&(pObjs[i].owner_cnt > 0)) || (pObjs[i].type == TARGET_TYPE_PEOPLE) ){
			pFinalObjs[pDep->n_final_objs] = pObjs[i];
			if( pDep->cam_height_adj > 0 )
				pFinalObjs[pDep->n_final_objs].height = pFinalObjs[pDep->n_final_objs].height * pDep->cam_height_adj / pDep->cam_height;
			pDep->n_final_objs ++;
		}
	}

	if( pDep->no_people_cnt == 10 )
		HandleTrack( pDep );
}

int CheckTrack( Depth * pDep, int idx )
{
	ObjectTrack * obj_track = pDep->obj_track;
	int obj_type = 0;// 0:新目标，1:已报过的目标，2:已报过的目标消失
	int merge_obj = -1;
	int min_life = pDep->ct_max_life + 1;
	int min_life_idx = -1;
	int choose_fg_obj = 0;
	int choose_bg_obj = 0;
	int j;
	//char tmp[64];
	for( j=0; j<MAX_OBJ_TRACK; j++ ){
		if( min_life > obj_track[j].life ){
			min_life     = obj_track[j].life;
			min_life_idx = j;
		}
		if( (obj_track[j].life > 0) && (DepthInnerDif(obj_track[j].rect, pDep->pObjs[idx].region) > 20) ){
			int diff_fg = Different( pDep, pDep->pFrameGrey, obj_track[j].fg, pDep->pObjs[idx].region );
			int diff_bg = Different( pDep, pDep->pFrameGrey, obj_track[j].bg, pDep->pObjs[idx].region );
			//sprintf(tmp, "b:\\track_%d_fg.bmp", j);
			//SaveGreyBitmap( obj_track[j].fg, pDep->width, pDep->height, tmp );
			//sprintf(tmp, "b:\\track_%d_bg.bmp", j);
			//SaveGreyBitmap( obj_track[j].bg, pDep->width, pDep->height, tmp );
			if( diff_bg < pDep->obj_track_th ){
				obj_type = 2;
				merge_obj = j;
				choose_fg_obj = diff_fg;
				choose_bg_obj = diff_bg;
				//printf("------------------------------------------------------------\n");
				break;
			}
			else if( (diff_fg < pDep->obj_track_th) && (diff_bg >= pDep->obj_track_th) ){
				obj_type = 1;
				merge_obj = j;
				choose_fg_obj = diff_fg;
				choose_bg_obj = diff_bg;
				//printf("------------------------------------------------------------\n");
				break;
			}
			if( obj_track[j].life > 0 )
				obj_track[j].life --;
		}
	}
	// deal with the object --------------------------------------------------
	switch( obj_type ){
	case 1:  // 已经报过的目标
		memcpy( obj_track[merge_obj].fg, pDep->pFrameGrey, pDep->frame_len );
		obj_track[merge_obj].rect = pDep->pObjs[idx].region;
		obj_track[merge_obj].life = pDep->ct_max_life;
		obj_track[merge_obj].fg_score = choose_fg_obj;
		obj_track[merge_obj].bg_score = choose_bg_obj;
		break;
	case 2:   // 消失的目标
		memcpy( obj_track[merge_obj].bg, pDep->pFrameGrey, pDep->frame_len );
		obj_track[merge_obj].fg_score = choose_fg_obj;
		obj_track[merge_obj].bg_score = choose_bg_obj;
		break;
	case 0:    // 新目标，覆盖最老的跟踪物体
	default:
		obj_track[min_life_idx].rect = pDep->pObjs[idx].region;
		obj_track[min_life_idx].fg_score = 0;
		obj_track[min_life_idx].bg_score = 0;
		obj_track[min_life_idx].life = pDep->ct_max_life;
		memcpy( obj_track[min_life_idx].fg, pDep->pFrameGrey, pDep->frame_len );
		GenBG( pDep );
		memcpy( obj_track[min_life_idx].bg, pDep->pFrameBG, pDep->frame_len );
		break;
	}
	return obj_type;
}

void HandleTrack( Depth * pDep )
{
	int j,dif_bg,dif_fg;
	// delete tracked obj that are disappear------------------------------------------------------------------------
	for( j=0; j<MAX_OBJ_TRACK; j++ ){
		if( pDep->obj_track[j].life > 0 ){
			dif_bg = Different( pDep, pDep->pFrameGrey, pDep->obj_track[j].bg, pDep->obj_track[j].rect );
			dif_fg = Different( pDep, pDep->pFrameGrey, pDep->obj_track[j].fg, pDep->obj_track[j].rect );

			if( !( (dif_bg > pDep->obj_track_th) && (dif_fg < pDep->obj_track_th) ) ){
				pDep->obj_track[j].life = 0;
			}
			else
				pDep->obj_track[j].life ++;
		}
	}
}

void GenBG( Depth * pDep )
{
	unsigned char * pGrey= pDep->pFrameBG;
	BkgModel * pModel = pDep->pModel;
	BkgModel * pModelE= pDep->pModel + pDep->frame_len * pDep->samples_per_pix;
	int accu_grey,i;
	while( pModel < pModelE ){
		accu_grey=0;
		for( i=0; i<pDep->samples_per_pix; i++ ){
			accu_grey += pModel->grey;
			pModel ++;
		}
		accu_grey /= pDep->samples_per_pix;
		pGrey[0] = (unsigned char)accu_grey;
		pGrey ++;
	}//*/
}

int  Different( Depth * pDep, unsigned char * l, unsigned char * r, Rect rect )
{
	int offset = rect.top * pDep->width;
	unsigned char * pL = l + offset;
	unsigned char * pR = r + offset;
	int i,j;
	int sum = 0;
	int total = (rect.bottom - rect.top) * (rect.right - rect.left);
	for( j=rect.top; j<rect.bottom; j++ ){
		for( i=rect.left; i<rect.right; i++ ){
			sum += abs(pL[i]-pR[i]);
		}
		pL += pDep->width;
		pR += pDep->width;
	}
	return sum / total;
}

#define STRIDE_LEN 4
float Inner( int * a, int * b, int len, float * the_dev )
{
	int div_len = len / STRIDE_LEN;
	int div_rst = len % STRIDE_LEN;
	__m128i sum = _mm_setzero_si128();
	__m128i dev = _mm_setzero_si128();
	__m128i * pa = (__m128i*)a;
	__m128i * pb = (__m128i*)b;
	__m128i * pae= pa + div_len;
	float sum_stride;
	int i;
	for( ; pa < pae; pa ++, pb ++ ){
		__m128i ia,ib,sub,add;
		ia = _mm_loadu_si128( pa );
		ib = _mm_loadu_si128( pb );
		add = _mm_add_epi32( ia, ib );
		sub = _mm_sub_epi32( ia, ib );
		sub = _mm_abs_epi32( sub );
		sum = _mm_add_epi32( sum, add );
		dev = _mm_add_epi32( dev, sub );
	}
	sum_stride = (float)(sum.m128i_i32[0] + sum.m128i_i32[1] + sum.m128i_i32[2] + sum.m128i_i32[3]);
	*the_dev = (float)(dev.m128i_i32[0] + dev.m128i_i32[1] + dev.m128i_i32[2] + dev.m128i_i32[3]);
	for( i=0; i<div_rst; i++ ){
		int idx = len - i - 1;
		sum_stride += a[idx] + b[idx];
		*the_dev   += abs(a[idx]-b[idx]);
	}
	return sum_stride;
}

Rect MergeRect( Rect l, Rect r )
{
	if( l.left > r.left )
		l.left = r.left;
	if( l.right < r.right )
		l.right = r.right;
	if( l.top > r.top )
		l.top = r.top;
	if( l.bottom < r.bottom )
		l.bottom = r.bottom;
	return l;
}

Rect AndRect( Rect lrect, Rect rrect )
{
	Rect common;
	if( lrect.left > rrect.left )
		common.left = lrect.left;
	else
		common.left = rrect.left;

	if( lrect.right < rrect.right )
		common.right = lrect.right;
	else
		common.right = rrect.right;

	if( lrect.top > rrect.top )
		common.top = lrect.top;
	else
		common.top = rrect.top;

	if( lrect.bottom < rrect.bottom )
		common.bottom = lrect.bottom;
	else
		common.bottom = rrect.bottom;
	return common;
}

int  CalcSpeed( Rect pre, Rect now, short * spx, short * spy )
{
	int lx,ly,rx,ry;
	int speed_x,speed_y;
	int min_dis,dis,speed;
	lx = (pre.left + pre.right) >> 1;
	ly = (pre.top + pre.bottom) >> 1;
	rx = (now.left + now.right) >> 1;
	ry = (now.top + now.bottom) >> 1;
	// minimal x speed------------------------
	speed_x = rx - lx;
	min_dis = abs(speed_x);
	speed   = now.left - pre.left;
	dis     = abs(speed);
	if( min_dis > dis ){
		speed_x = speed;
		min_dis = dis;
	}
	speed    = now.right - pre.right;
	dis      = abs(speed);
	if( min_dis > dis ){
		speed_x = speed;
		min_dis = dis;
	}
	// minimal y speed-------------------------
	speed_y = ry - ly;
	min_dis = abs(speed_y);
	speed   = now.top - pre.top;
	dis     = abs(speed);
	if( min_dis > dis ){
		speed_y = speed;
		min_dis = dis;
	}
	speed    = now.bottom - pre.bottom;
	dis      = abs(speed);
	if( min_dis > dis ){
		min_dis = dis;
		speed_y = speed;
	}
	if( spx )
		*spx = speed_x;
	if( spy )
		*spy = speed_y;
	return (int)sqrt( (float)(speed_x*speed_x + speed_y*speed_y) );
}

int RectDistance( Rect lrect, Rect rrect )
{
	Rect common;
	int l_span,r_span;
	if( lrect.left > rrect.left )
		common.left = lrect.left;
	else
		common.left = rrect.left;

	if( lrect.right < rrect.right )
		common.right = lrect.right;
	else
		common.right = rrect.right;

	if( lrect.top > rrect.top )
		common.top = lrect.top;
	else
		common.top = rrect.top;

	if( lrect.bottom < rrect.bottom )
		common.bottom = lrect.bottom;
	else
		common.bottom = rrect.bottom;
	l_span = common.right - common.left;
	if( l_span > 0 )
		l_span = 0;
	else
		l_span = -l_span;
	r_span = common.bottom - common.top;
	if( r_span > 0 )
		r_span = 0;
	else
		r_span = -r_span;
	if( l_span > r_span )
		return l_span;
	else
		return r_span;
}

void DepthGenGrad( Depth * pDep )
{
	unsigned char * pFrameGreyUp = pDep->pFrameGrey + 1;
	unsigned char * pFrameGreyCenter = pDep->pFrameGrey + pDep->width + 1;
	unsigned char * pFrameGreyDown   = pDep->pFrameGrey + (pDep->width << 1) + 1;
	unsigned char  * pFrameGradUp    = pDep->pFrameGrad  + 1;
	unsigned char  * pFrameGrad      = pDep->pFrameGrad  + pDep->width + 1;
	unsigned char  * pFrameGradDown  = pDep->pFrameGrad  + (pDep->width<<1) + 1;
	int i,j;
	int GradX,GradY;
	unsigned char grey_mask;
	memset( pDep->pFrameGrad, 0, pDep->frame_len );
	for( i=1; i<pDep->height-1; i++ ){
		for( j=1; j<pDep->width-1; j++ ){
			/* Sobel for edge detection*/
			GradX = ((int)pFrameGreyUp[1]-(int)pFrameGreyUp[-1])+
			((int)(pFrameGreyCenter[1]<<1) - (int)(pFrameGreyCenter[-1]<<1))+
			((int)pFrameGreyDown[1]-(int)pFrameGreyDown[-1]);
			GradY = ((int)pFrameGreyDown[-1] - (int)pFrameGreyUp[-1])+
			((int)(pFrameGreyDown[0]<<1) - (int)(pFrameGreyUp[0]<<1))+
			((int)pFrameGreyDown[1]-(int)pFrameGreyUp[1]);
			/* Edge Detect */
			//grey_mask = ((GradX*GradX+GradY*GradY)*5 > (int)pFrameGreyCenter[0]*(int)pFrameGreyCenter[0])?255:0;
			grey_mask = (unsigned char)((abs(GradX) + abs(GradY))>>3);
			GradX = abs(GradX);
			GradY = abs(GradY);
			if( GradX > GradY )
				grey_mask = GradX >> 2;
			else
				grey_mask = GradY >> 2;

			*pFrameGrad    = grey_mask;
			//pFrameGrad[-1] = grey_mask;
			//pFrameGrad[1]  = grey_mask;
			//*pFrameGradUp    = grey_mask;
			//pFrameGradUp[-1] = grey_mask;
			//pFrameGradUp[1]  = grey_mask;
			//*pFrameGradDown    = grey_mask;
			//pFrameGradDown[-1] = grey_mask;
			//pFrameGradDown[1]  = grey_mask;

			pFrameGreyUp ++;
			pFrameGreyCenter ++;
			pFrameGreyDown ++;
			pFrameGrad ++;
			pFrameGradUp ++;
			pFrameGradDown ++;
		}
		pFrameGreyUp += 2;
		pFrameGreyCenter += 2;
		pFrameGreyDown += 2;
		pFrameGrad += 2;
		pFrameGradUp += 2;
		pFrameGradDown += 2;
	}
}

void HandleRegionPeopleCnt( Depth * pDep, Region * pregion, int people_cnt )
{
	pregion->is_evt = 0;
	if( people_cnt != pregion->pre_people_cnt ){
		pregion->pre_cnt_frames = 0;
		pregion->pre_people_cnt = people_cnt;
	}
	else{
		if( pregion->pre_cnt_frames <= pDep->region_pre_cnt_th )
			pregion->pre_cnt_frames ++;
	}

	if( pregion->pre_cnt_frames >= pDep->region_pre_cnt_th ){
		if( people_cnt > pregion->cur_people_cnt ){
			if( pregion->type == REGION_TYPE_VALID )
				pregion->is_evt |= EVT_REGION_OBJ_IN;
			if( pregion->type == REGION_TYPE_METER ){
				if( people_cnt > pregion->people_up ){
					pregion->status = REGION_STATUS_VIOLATE;
					if( pregion->cur_cnt_frames > pDep->region_in_vio_th )
						pregion->is_evt |= EVT_REGION_IN_VIO;
					else
						pregion->is_evt |= EVT_REGION_CNT_VIO_UP;
				}
				else if( people_cnt < pregion->people_down )
					pregion->status = REGION_STATUS_VIOLATE;
				else{
					if( pregion->status != REGION_STATUS_NORMAL )
						pregion->is_evt |= EVT_REGION_CNT_NORMAL;
					pregion->status = REGION_STATUS_NORMAL;
				}
			}
			pregion->cur_cnt_frames = 0;
		}
		else if( people_cnt < pregion->cur_people_cnt ){
			if( pregion->type == REGION_TYPE_VALID )
				pregion->is_evt |= EVT_REGION_OBJ_OUT;
			pregion->cur_cnt_frames = 0;
			if( pregion->type == REGION_TYPE_METER ){
				if( ((people_cnt <= pregion->people_up)
					&& (people_cnt >= pregion->people_down))
					|| (people_cnt == 0)
					){
					if( pregion->status != REGION_STATUS_NORMAL )
						pregion->is_evt |= EVT_REGION_CNT_NORMAL;
					pregion->status = REGION_STATUS_NORMAL;
				}
				else
					pregion->status = REGION_STATUS_VIOLATE;
			}
		}
		else{
			if( pregion->cur_cnt_frames < 1000 )
				pregion->cur_cnt_frames ++;
			if( (pregion->cur_people_cnt < pregion->people_down)
				&& (pregion->cur_people_cnt > 0)
				&& (pregion->cur_cnt_frames == pDep->region_fbuf_th)
				){
					if( pregion->type == REGION_TYPE_METER )
						pregion->is_evt |= EVT_REGION_CNT_VIO_DOWN;
			}
		}
		pregion->cur_people_cnt = people_cnt;
	}
	else{
		if( pregion->cur_cnt_frames < 1000 )
			pregion->cur_cnt_frames ++;
	}
}

int  AcutanceAndGrey( Depth * pqua, unsigned char * pFrameGrey )
{
	//清晰度异常检测
	int z9,z1,z2,z3,z4,z5,z6,z7,z8,i;
    int sum=0;
	int grey=0;
	int n = pqua->width * pqua->height;
	unsigned char * pUpper  = pFrameGrey;
	unsigned char * pCenter = pFrameGrey + pqua->width;
	unsigned char * pLower  = pFrameGrey + (pqua->width<<1);
	unsigned char * pEnd    = pFrameGrey + n - pqua->width;
    for(;pCenter<pEnd;pCenter+=pqua->width)
	{
        for(i=1;i<pqua->width-1;i++)
		{
	        z1    = abs(pUpper[i-1]	 - pCenter[i]);
            z2    = abs(pUpper[i]	 - pCenter[i]);
            z3    = abs(pUpper[i+1]	 - pCenter[i]);
            z4    = abs(pCenter[i-1] - pCenter[i]);
            z5    = abs(pCenter[i+1] - pCenter[i]);
            z6    = abs(pLower[i-1]	 - pCenter[i]);
            z7    = abs(pLower[i]	 - pCenter[i]);
            z8    = abs(pLower[i+1]	 - pCenter[i]);
            z9    = z1+z2+z3+z4+z5+z6+z7+z8;
            sum  += z9;
			grey += pCenter[i];
		}
		pUpper += pqua->width;
		pCenter+= pqua->width;
		pLower += pqua->width;
	}
	pqua->acutance = sum/n;
    return grey/n;
}

void FilterOutDepth( Depth * pDep, unsigned short * pDepth )
{
	int i,j;
	int idx = pDep->cur_dep_buf;
	int avg,avg_cnt,up,down,left,right,cnt,sum=0;

	// buffer the frame----------------------------------------------------------
	memcpy( pDep->pFrameDepth, pDepth, pDep->frame_len*sizeof(short));
	//*
	// test the data-------------------------------------------------------------
	for( i=pDep->width; i<pDep->frame_len - pDep->width; i++ ){
		if( pDepth[i] == 0 )
			continue;
		avg = 0, avg_cnt = 0;
		for( j=0; j<DEP_BUF_SIZE; j++ ){
			if( pDep->pDepBuf[j][i] > 0 ){
				avg += pDep->pDepBuf[j][i];
				avg_cnt ++;
			}
		}
		if( avg_cnt > 1 )
			avg /= avg_cnt;
		else
			continue;

		if( abs( avg - pDepth[i] ) > pDep->odd_pt_th ){
			up  = pDepth[i-pDep->width];
			down= pDepth[i+pDep->width];
			left= pDepth[i-1];
			right=pDepth[i+1];
			cnt = 0;
			if( abs( up - pDepth[i] ) > pDep->odd_pt_th )
				cnt ++;
			if( abs( down - pDepth[i] ) > pDep->odd_pt_th )
				cnt ++;
			if( abs( left - pDepth[i] ) > pDep->odd_pt_th )
				cnt ++;
			if( abs( right - pDepth[i] ) > pDep->odd_pt_th )
				cnt ++;
			if( cnt >= 2 )
				pDep->pFrameDepth[i] = 0;
				sum ++;
		}
	}

	// update the buf-------------------------------------------------------------
	memcpy( pDep->pDepBuf[idx], pDepth, sizeof(unsigned short)*pDep->frame_len );
	pDep->cur_dep_buf = (idx + 1) % DEP_BUF_SIZE;
	//*/
}

int FloorDif( Depth * pDep, Rect curr, Rect last )
{
	int i,j;
	int c_x,c_y,l_x,l_y;
	int c_w,c_h,l_w,l_h,width,height;
	FloorAttr * pFAttr = pDep->pFloorAttr;
	FloorAttr * pFAttrL= pDep->pFloorAttrLast;
	long long sum, cnt;
	c_x = (curr.left + curr.right - 1) >> 1;
	c_y = (curr.top  + curr.bottom- 1) >> 1;
	l_x = (last.left + last.right - 1) >> 1;
	l_y = (last.top  + last.bottom- 1) >> 1;
	c_w = curr.right - curr.left;
	c_h = curr.bottom- curr.top;
	l_w = last.right - last.left;
	l_h = last.bottom- last.top;
	width = c_w;
	if( width < l_w )
		width = l_w;
	height = c_h;
	if( height < l_h )
		height = l_h;
	height= curr.bottom - curr.top;
	curr.left = c_x - (width>>1);
	curr.right= c_x + (width>>1);
	curr.top  = c_y - (height>>1);
	curr.bottom=c_y + (height>>1);
	last.left = l_x - (width>>1);
	last.right= l_x + (width>>1);
	last.top  = l_y - (height>>1);
	last.bottom=l_y + (height>>1);
	if( (curr.left < 0) || (last.left < 0) || (curr.right >= pDep->floor_attr_width) || (last.right >= pDep->floor_attr_width) )
		return -1;
	if( (curr.top < 0) || (last.top < 0) || (curr.bottom >= pDep->floor_attr_height) || (last.bottom >= pDep->floor_attr_height) )
		return -1;
	pFAttr = pDep->pFloorAttr + curr.top * pDep->floor_attr_width + curr.left;
	pFAttrL= pDep->pFloorAttrLast + last.top * pDep->floor_attr_width + last.left;
	sum = cnt = 0;
	for( i=0; i<height; i++ ){
		for( j=0; j<width; j++ ){
			if( (pFAttr[j].prj_cnt>0) && (pFAttrL[j].prj_cnt>0) ){
				sum += abs(pFAttr[j].max_height - pFAttrL[j].max_height);
				cnt ++;
			}
		}
		pFAttr += pDep->floor_attr_width;
		pFAttrL+= pDep->floor_attr_width;
	}
	if( cnt > 0 )
		return (int)(cnt*100/(width*height));
	else
		return -1;
}

bool  IsCross(Point pt1, Point pt2, Point line_s, Point line_e, Point dir)
{
	Rect r_pt, r_l, common;
	Point pt1_s,pt1_e,pt2_s,pt2_e,dir_s,dir_e;
	int cross1,cross2,cross3;
	// 检测量线段为对角线的矩形是否相交
	if( pt1.x < pt2.x ){
		r_pt.left = pt1.x;
		r_pt.right = pt2.x + 1;
	}
	else{
		r_pt.left = pt2.x;
		r_pt.right = pt1.x + 1;
	}
	if( pt1.y < pt2.y ){
		r_pt.top = pt1.y;
		r_pt.bottom=pt2.y + 1;
	}
	else{
		r_pt.top = pt2.y;
		r_pt.bottom=pt1.y + 1;
	}
	if( line_s.x < line_e.x ){
		r_l.left = line_s.x;
		r_l.right= line_e.x + 1;
	}
	else{
		r_l.left = line_e.x;
		r_l.right= line_s.x + 1;
	}
	if( line_s.y < line_e.y ){
		r_l.top = line_s.y;
		r_l.bottom=line_e.y;
	}
	else{
		r_l.top = line_e.y;
		r_l.bottom=line_s.y + 1;
	}
	if( r_pt.left < r_l.left )
		common.left = r_l.left;
	else
		common.left = r_pt.left;
	if( r_pt.right < r_l.right )
		common.right = r_pt.right;
	else
		common.right = r_l.right;
	if( r_pt.top < r_l.top )
		common.top = r_l.top;
	else
		common.top = r_pt.top;
	if( r_pt.bottom < r_l.bottom )
		common.bottom = r_pt.bottom;
	else
		common.bottom = r_l.bottom;
	if( (common.left >= common.right) || (common.bottom >= common.top) )
		return false;

	// 测试顺逆方向
	pt1_s.x = line_s.x - pt1.x;
	pt1_s.y = line_s.y - pt1.y;
	pt1_e.x = line_e.x - pt1.x;
	pt1_e.y = line_e.y - pt1.y;
	pt2_s.x = line_s.x - pt2.x;
	pt2_s.y = line_s.y - pt2.y;
	pt2_e.x = line_e.x - pt2.x;
	pt2_e.y = line_e.y - pt2.y;
	dir_s.x = line_s.x - dir.x;
	dir_s.y = line_s.y - dir.y;
	dir_e.x = line_e.x - dir.x;
	dir_e.y = line_e.y - dir.y;

	cross1 = pt1_s.x*pt1_e.y - pt1_s.y*pt1_e.x;
	cross2 = pt2_s.x*pt2_e.y - pt2_s.y*pt2_e.x;
	cross3 = dir_s.x*dir_e.y - dir_s.y*dir_e.x;

	if( cross1*cross2 > 0 )
		return false;

	if( cross2*cross3 < 0 )
		return false;

	return true;
}

void World2UV( Depth * pDep, int * u, int * v, int x, int y, int h )
{
	float inv_u, inv_v, inv_d;
	inv_u = x * pDep->invX[0] + y * pDep->invX[1] + (h-pDep->cam_height)*pDep->invX[2];
	inv_v = x * pDep->invY[0] + y * pDep->invY[1] + (h-pDep->cam_height)*pDep->invY[2];
	inv_d = x * pDep->invZ[0] + y * pDep->invZ[1] + (h-pDep->cam_height)*pDep->invZ[2];
	*u = (int)(inv_u*pDep->fu/inv_d+(pDep->width>>1));
	*v = (int)(inv_v*pDep->fv/inv_d+(pDep->height>>1));
	if( *u < 0 )
		*u = 0;
	if( *u >= pDep->width )
		*u = pDep->width - 1;
	if( *v < 0 )
		*v = 0;
	if( *v >= pDep->height )
		*v = pDep->height - 1;
}

void CheckCamQuality( Depth * pDep )
{
	long long avg_depth = 0;
	int i;
	int nxt_grey, grey_dif, avg_grey;
	unsigned short * pDepth = pDep->pFrameDepth;

	pDep->cam_evt = 0;

	avg_grey = AcutanceAndGrey( pDep, pDep->pFrameGrey );
	pDep->avg_grey[pDep->cur_avg_grey] = avg_grey;
	nxt_grey = (pDep->cur_avg_grey + 1)%MAX_GREY_HISTORY;
	grey_dif = pDep->avg_grey[pDep->cur_avg_grey] - pDep->avg_grey[nxt_grey];
	pDep->cur_avg_grey = nxt_grey;

	// 摄像头致命状态异常检测==================================================================
	// 亮度过高/过低的检测----------------------------------------------------------------
	if( avg_grey > pDep->light_up_th ){
		pDep->cur_status = DEPTH_STATUS_FATAL;
	}
	if( avg_grey < pDep->light_down_th ){
		pDep->cur_status = DEPTH_STATUS_FATAL;
	}
	// 锐度异常检测-------------------------------------------------------------------------
	if( pDep->acutance < pDep->blur_thresh ){
		pDep->cur_status = DEPTH_STATUS_FATAL;
	}
	// 深度值的有效个数和平均深度过低检测----------------------------------------------------
	pDep->dep_valid_cnt = 0;
	avg_depth = 0;
	for( i=0; i<pDep->frame_len; i++ ){
		if( pDepth[i] > 0 ){
			avg_depth += pDepth[i];
			pDep->dep_valid_cnt ++;
		}
	}
	if( pDep->dep_valid_cnt > 0 )
		pDep->avg_depth = (int)(avg_depth / pDep->dep_valid_cnt);
	else
		pDep->avg_depth = 0;
	if( pDep->avg_depth < pDep->cover_thresh ){
		pDep->cur_status = DEPTH_STATUS_FATAL;
	}
	if( pDep->cur_status == DEPTH_STATUS_FATAL ){
		if( pDep->cam_fatal_cnt <= pDep->cover_span )
			pDep->cam_fatal_cnt ++;
	}
	if( pDep->cur_status == DEPTH_STATUS_FATAL ){
		if( pDep->cam_fatal_cnt <= pDep->cover_span )
			pDep->cam_fatal_cnt ++;
		pDep->cam_slash_cnt = pDep->cover_span;
	}
	else
		pDep->cam_fatal_cnt = 0;
	if( pDep->cam_fatal_cnt == pDep->cover_span )
		pDep->cam_evt |= EVT_CAM_COVER_BLUR_DARK_BRIGHT;
	// 摄像头致命状态异常检测结束==================================================================

	// 摄像头瞬时异常检测================================================================
	// 亮度变化检测------------------------------------------------------------------
	if( pDep->cur_status != DEPTH_STATUS_FATAL ){
		if( grey_dif > pDep->light_thresh ){
			if( pDep->cam_slash_cnt == 0 ){
				pDep->cam_evt |= EVT_CAM_MOVED_LIGHT_UP_DOWN;
			}
			pDep->cam_slash_cnt = pDep->cover_span;
		}
		else if( grey_dif < -pDep->light_thresh ){
			if( pDep->cam_slash_cnt == 0 ){
				pDep->cam_evt |= EVT_CAM_MOVED_LIGHT_UP_DOWN;
			}
			pDep->cam_slash_cnt = pDep->cover_span;
		}
		// 摄像头移动检测------------------------------------------------------------------
		if( pDep->dfg_dif_percent > pDep->cam_move_thresh ){
			if( pDep->cam_slash_cnt == 0 )
				pDep->cam_evt |= EVT_CAM_MOVED_LIGHT_UP_DOWN;
			pDep->cam_slash_cnt = pDep->cover_span;
			pDep->cam_reinit_cnt = pDep->frame_ignore;
		}
	}
	if( pDep->cam_slash_cnt > 0 )
		pDep->cam_slash_cnt --;
	if( pDep->cam_reinit_cnt > 0 )
		pDep->cam_reinit_cnt --;
	if( pDep->cam_reinit_cnt == 1 )
		pDep->frame_cnt = pDep->frame_ignore;
	if( pDep->cur_status == DEPTH_STATUS_FATAL )
		pDep->cam_reinit_cnt = 0;
	// 摄像头瞬时异常检测结束==============================================================

	// 摄像头病态错误检测====================================================================
	if( pDep->cur_status != DEPTH_STATUS_FATAL ){
		if( (pDep->dep_valid_cnt << 7) < pDep->dep_valid_thresh * pDep->frame_len ){
			pDep->cur_status = DEPTH_STATUS_ILL;
			if( pDep->cam_sun_cnt <= pDep->cover_span )
				pDep->cam_sun_cnt ++;
		}
		else{
			pDep->cam_sun_cnt = 0;
		}
		if( pDep->cam_sun_cnt == pDep->cover_span ){
			pDep->cam_evt |= EVT_CAM_DEP_VALID_SMALL;
		}

		// 地面点个数过低检测------------------------------------------------------------
		if( ((pDep->up_pt_cnt << 7) < pDep->dep_valid_cnt * pDep->floor_cnt_thresh) ){
			pDep->cur_status = DEPTH_STATUS_ILL;
			if( pDep->cam_nofloor_cnt <= pDep->cover_span )
				pDep->cam_nofloor_cnt ++;
		}
		else
			pDep->cam_nofloor_cnt = 0;
		if( pDep->cam_nofloor_cnt == pDep->cover_span )
			pDep->cam_evt |= EVT_CAM_NOFLOOR;
	}
	// 摄像头病态错误检测结束====================================================================
}

int  IsInRect( int u, int v, Rect rct )
{
	return (u>=rct.left)&&(u<rct.right)&&(v>rct.top)&&(v<rct.bottom);
}
