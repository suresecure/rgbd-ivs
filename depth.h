#ifndef DEPTH_H_
#define DEPTH_H_

#include"common.h"
#include"util.h"

#ifdef __cplusplus
extern "C"{
#endif

#define MAX_TARGET_HISTORY 5
#define MAX_WANDER_HISTORY 5
#define MAX_OBJ_TRACK 16
#define DEP_BUF_SIZE  3
#define MAX_GREY_HISTORY 10
#define MAX_PLANE_PTS 10000

#define DEPTH_STATUS_INIT      0   // ��ʼ��״̬
#define DEPTH_STATUS_NORM      1   // ��������״̬
#define DEPTH_STATUS_FATAL     2   // ����ͷ��������״̬������״̬�²������κη����������������������ڵ���ģ��
#define DEPTH_STATUS_ILL       3   // ����������״̬����Ҫ������Ч���ֵ���٣�û�п�������
#define DEPTH_STATUS_ERROR     4   // ����״̬��ģ�鷢���˴���

#define REGION_MAX_CNT         8  // ��ģ������ܹ����õ��������
#define REGION_MAX_PTS         16 // ÿ����������������������

#define REGION_STATUS_UNSET      0  // ��Ч״̬
#define REGION_STATUS_NORMAL     1  // ����״̬
#define REGION_STATUS_VIOLATE    2  // Υ��״̬

#define REGION_TYPE_NORMAL      0  // ���������������κ��¼�
#define REGION_TYPE_METER       1  // ��������������������������һ����Χ֮�ڣ������¼����������ࡢ�������١������ظ�����
#define REGION_TYPE_CROSS       2  // ������������ָ��ĳ�����ߵġ��ߡ��ͷ��������¼�����Ա����
#define REGION_TYPE_VALID       3  // ��Ч����Ŀ��Ҫ����������ĳ����Ч���ڣ������¼�����Ա���롢��Ա�뿪
#define REGION_TYPE_DOOR        4  // �ż���������ĳ���ŵĴ򿪡��ر�
#define REGION_TYPE_VIO         5  // very important object���������Է���Ҫ��Ʒ��ʧ

#define EVT_LIEDOWN       1   // Ŀ�굹���¼�
#define EVT_RUNNING       2   // Ŀ���ܲ��¼�
#define EVT_LIEDOWN_LONG  4   // Ŀ�곤ʱ�䵽���¼��������ε���
#define EVT_FIGHT         8   // Ŀ�����¼�
#define EVT_STAY          16  // Ŀ�������¼��������������������
#define EVT_WANDER        32  // Ŀ���ǻ��¼������ﲻ�����£����������˶���
#define EVT_STILL         64  // ��Ʒ�����¼������屣�ֲ�����
#define EVT_DOWN          128 // Ŀ���¶��¼�
#define EVT_PLANE_STILL   256 // ճ���������¼���ƽ�����壬��ȫ��rgb���õ������壩
#define EVT_SAMPLE_FRAME  512 // �����ɼ��¼�

#define EVT_CAM_COVER_BLUR_DARK_BRIGHT      1   // ����ͷ�����쳣���ڵ���ģ��������������
#define EVT_CAM_MOVED_LIGHT_UP_DOWN         2   // ����ͷ˲ʱ�쳣���ƶ�������ͻ��
#define EVT_CAM_NOFLOOR                     4   // ����ͷû�п�������
#define EVT_CAM_DEP_VALID_SMALL             8   // ��Ч���ֵ����
#define EVT_CAM_SOMEFOLLOW                  16  // β���¼�����Ϊ����ͷ�¼�����
#define EVT_CAM_FIGHT                       32  // ����¼���Ҳ��Ϊ����ͷ����

#define EVT_REGION_CNT_VIO_UP    1   // ��������Υ���¼�(�������ָ࣬�������˽��룩
#define EVT_REGION_CNT_VIO_DOWN  2   // ��������Υ���¼�(�������٣�
#define EVT_REGION_OBJ_IN        4   // �������˽����¼�
#define EVT_REGION_OBJ_OUT       8   // ���������뿪�¼�
#define EVT_REGION_IN_VIO        16  // ��������Υ����루ǿ�����뵼���������ָ࣬�����̶�һ��ʱ��������˽��뵼���������ࣩ
#define EVT_REGION_CNT_NORMAL    32  // ���������ָ������¼�
#define EVT_REGION_CROSS         64  // �������˴�����ʵ���ǰ������˴�����
#define EVT_CAM_SINGLE_OPEN      128 // �����˼ӳ�������ֻ��һ�ˣ��ҳ����Ŵ򿪻��ڵ�

#define PT_TYPE_INVALID  0       // ��Ǹõ�����Ч����������
#define PT_TYPE_DVALID   1       // ��Ǹõ�����ֵ����Ч��
#define PT_TYPE_FLOOR    2       // ��Ǹõ��ǵ����
#define PT_TYPE_FG       4       // ��Ǹõ���ǰ����
#define PT_TYPE_FG_DEP   8       // ��Ǹõ�Ϊ��ȼ��ǰ��
#define PT_TYPE_FG_RGB   16      // ��Ǹõ�Ϊ��ɫ���ǰ��
#define PT_TYPE_FG_GRAD  32      // ��Ǹõ�Ϊ�ݶȼ��ǰ���㣬���ݶ�ǰ����ĵ�һ��Ҳ�ǲ�ɫ���ǰ����
#define PT_TYPE_SHADOW   64      // ��Ǹõ�����Ӱ��
#define PT_TYPE_SUPPORT  128     // ��Ǹõ���֧�ŵ�
#define PT_TYPE_FG2BG    256     // ��Ǹõ���ǰ��ת����
#define PT_TYPE_FG_EXT   512     // ��Ǹõ��Ǿ���Refine���ǰ��

#define TARGET_TYPE_NORMAL 0     // �ոռ���������������
#define TARGET_TYPE_PEOPLE 1     // �ж�Ϊ��Ա������
#define TARGET_TYPE_FLOOR  2     // ��ȫ�ɹ�ѧ���������壬��һ���ڵ�����
#define TARGET_TYPE_WALL   3     // �ж�Ϊǽ��
#define TARGET_TYPE_FLOAT  4     // ֧�Ų�������壨�������壩

#define TARGET_STATUS_LIEDOWN      1   // ����״̬
#define TARGET_STATUS_MOVE         2   // �ƶ�״̬
#define TARGET_STATUS_FOLLOW       4   // β��״̬
#define TARGET_STATUS_STAY_STILL   16  // ����״̬������/���壩
#define TARGET_STATUS_DOWN         64  // �¶�״̬

#define PLANE_TYPE_NO    0  //����ƽ���
#define PLANE_TYPE_UP    1  //�����ϵ�ƽ���
#define PLANE_TYPE_HZ    2  //����ˮƽ��ƽ���
#define PLANE_TYPE_OTHER 3  //���򲻹����ƽ���

	// the background model struct for the depth camera
	typedef struct BkgModel_{
		unsigned char  grey;
		unsigned char  r;
		unsigned char  g;
		unsigned char  b;
		unsigned short d;
		unsigned char grad;
	} BkgModel;

	typedef struct FloorAttr_{
		short min_height;         // Ͷ�䵽��λ�õ���͸߶�
		short min_u,min_v;        // ��͵�ͶӰ��ͼ������
		short max_height;         // Ͷ�䵽��λ�õ���߸߶�
		short max_u,max_v;        // ��ߵ�ͶӰ��ͼ������
		short prj_cnt;            // ͶӰ����λ�õ�ͼ���ĸ���

		short id;                 // ������ʱ�õ��ID��
		short polar_id;           // polar������ʱ�õ��ID��
		short polar_dis;          // polar������Ⱦ���õ㻨�ѵ�����
	} FloorAttr;

	typedef struct FloorTarget_{
		short max_height;         // ��Ŀ����ߵ�߶�
		short x,y;                // ��Ŀ���λ��
		Rect  rect;               // ��Ŀ����ռ��ͼ������
		short id;                 // ��Ŀ���ID
		int   mass;               // ��Ŀ���������ĵ����
		int   floor_mass;         // ��Ŀ���������Ľ�������ĵ�ĸ���

		short children_cnt;       // ��Ŀ�����Ŀ�����
	} FloorTarget;

	typedef struct PolarTarget_{
		short max_height;         // ��Ŀ����ߵ�߶�
		short x,y;                // ��Ŀ���λ��

		short avg_height;         // ��Ŀ����������ƽ���߶�
		Rect  rect;               // ��Ŀ����ռ��ͶӰͼ������
		Rect  rect_world;         // ��Ŀ����ռ��ʵ������
		Rect  rect_image;         // ��Ŀ����ռ��ͼ������

		short root;               // ��Ŀ�����ڵĴ�Ŀ��

		int   mass;               // ��Ŀ���������ĵ����
		int   floor_mass;         // ��Ŀ���������Ľ�������ĵ�ĸ���
		int   img_mass;           // ��Ŀ���ͼ�ϵ�ĸ���
		int   support_mass;       // ��Ŀ���֧�ŵ�

		// ��RefineTarget��������Ϣ
		int   dep_mass;
		int   prj_mass; 
	} PolarTarget;

	typedef struct PolarPoint_{
		short x,y;
	} PolarPoint;

	typedef struct PolarQueue_{
		int head;
		int tail;
		PolarPoint * queue;
	} PolarQueue;

	typedef struct RefineTarget_{
		short max_height;         // ��Ŀ����ߵ�߶�
		int   x,y;                // ��Ŀ���ʵ��λ�ã���ߵ㣩
		Rect  rect;               // ��Ŀ����ռ��ͼ������
		Rect  rect_world;         // ��Ŀ����ռ��ʵ������
		short id;                 // ��Ŀ������ID��
		short children_cnt;       // ��Ŀ���ڵ�PolarĿ��ĸ���

		int   mass;               // ��Ŀ���������ĵ����
		int   active_mass;        // ��Ŀ������Ļ�Ծ�����
		int   dep_mass;           // ��Ŀ��������������Ϣ�����ĵ�ĸ���
		int   grad_mass;          // ��Ŀ����������ݶ���Ϣ�����ĵ�ĸ���
		int   support_mass;       // ��Ŀ�������֧�ŵ����
		int   floor_mass;         // ��Ŀ������ĵ�������
		int   prj_mass;           // ��Ŀ�������ͶӰ�����
		char* low_pt;             // ��Ŀ������·���������Ϣ
		int*  rid_cnt;            // ��Ŀ���ridӳ�主����
	} RefineTarget;

	typedef struct ImgAttr_{
		short depth;               // �õ�����ֵ
		short x,y;                 // �õ��x,y���꣨��������ϵ��
		short height;              // �õ��ʵ�ʸ߶�
		short type;                // �õ������
		short active_cnt;          // ��Ծ����������õ㱻ĳ���㸲�ǣ���õ�����Ծ״̬
		short plane_type;          // �õ��ƽ������
		short is_in_vir;           // �Ƿ���VIR�У�very important region��
		short avg_bg_dep;          // �õ�ı�����ƽ�����
		short bg_height;           // ��¼�õ�������ĸ߶�

		short id;                  // �õ��ID
		short rid;                 // ��ӳ��õ���ID
		short polar_id;            // ��ӳ��õ���polar_id
		int   offset;              // �õ�ӳ�䵽ƽ��ͼ������
	} ImgAttr;

	typedef struct BGTarget_{
		Rect rect;                 // ��Ŀ���ͼ������
		int mass;                  // ��Ŀ����ܵ���
	} BGTarget;

	typedef struct ImgPlaneAttr_{
		float x,y,z;               // �õ��x,y,z����
		float height;              // �߶�
		short plane_type;          // �õ��ƽ������
	} ImgPlaneAttr;

	typedef struct Plane_{
		// the coeffs----------------
		float a;
		float b;
		float c;
		float d;

		// collected data------------
		int   total;
		int   pt_cnt;
		short pts_uv[MAX_PLANE_PTS][2];
		float pts[MAX_PLANE_PTS][3];
		float dir[MAX_PLANE_PTS][3];
	} Plane;

	typedef struct DepthRegion_{

		short u[REGION_MAX_PTS]; // ��Ǹ�������ĸ����u����
		short v[REGION_MAX_PTS]; // ��Ǹ�������ĸ����v����
		short x[REGION_MAX_PTS]; // ��Ǹ�������ĸ����x����
		short y[REGION_MAX_PTS]; // ��Ǹ�������ĸ����y����
		short n_pt;              // �ö������������ĵ���

		short type;              // �ö���ε�����

		short people_up;         // ��Ǹ�������Ա���������
		short people_down;       // ��Ǹ�������Ա����С����

		char  status;            // ��Ǹ������״̬
		short is_evt;            // ��Ǹ������Ƿ�Ҫ���¼�������һ��mask�͵ı���

		char  cur_people_cnt;    // ��Ǹ�����ĵ�ǰ����
		short cur_cnt_frames;    // ��Ǹ���ά�����ڵ�������֡��
		char  pre_people_cnt;    // ������������Ҫ��ɶ���
		short pre_cnt_frames;    // ���pre_people_cntά�ֵ�ǰֵ��֡��

		int   dfg_cnt;           // ���������ǰ����ĸ���
		short big_fg_frames;     // ����ǰ��������֡��
	} Region;

	typedef struct _ObjectTrack{
		Rect rect;
		unsigned char * bg;   // ��ǰ����ı���ͼƬ
		unsigned char * fg;   // ��ǰ�����ǰ��ͼƬ
		int life;        // ��ǰ������ٵ�����ֵ

		// ��ʱ����
		int fg_score;    // ���һ�κϲ�������ǰ������
		int bg_score;    // ���һ�κϲ������ı�������
	} ObjectTrack;

	// ���ڼ�⵽��������Ҫ��¼������---------------------------------------------
	typedef struct DepObject_{
		Rect  region;                              // �������������Ӿ��ο�
		Rect  region_world;                        // ��������ռ����
		Rect  lregion;                             // ��Ŀ����һ֡��λ��
		short speed_x,speed_y;                     // �����ϵ��ٶ�
		short max_move_cnt;                        // ��¼Ŀ�������ﵽ������ٶ�
		int   speed;                               // ��¼Ŀ�굱ǰ���ٶ�

		Rect  region_prj;                          // ��¼��Ŀ��߳�ͼ��λ��
		Rect  region_prj_last;                     // ��¼��Ŀ����һ�εĸ߳�ͼλ��
		Point region_floor[4];                     // �����ϵ�ͶӰ��rgbͼ�ϵ�����
		Point region_head[4];                      // �����ͷ����rgbͼ�ϵ�����

		short ori_cx, ori_cy;                      // Ŀ��ĳ�ʼλ��
		int   height;                              // ���嵱ǰ�߶�
		int   max_height;                          // Ŀ����ʷ�߶����ֵ
		int   cx, cy;                              // ��ߵ��λ��
		int   cwidth,cheight;                      // ��ߵ�ʱĿ��Ŀ�Ⱥ͸߶�
		short life;                                // ���������ֵ
		int   det_cnt;                             // ���屻���ٵ�֡��

		int   mass;                                // ��������������ǰ�������
		int   support_mass;                        // ������֧�ŵ����
		int   dep_mass;                            // ����������ǰ�������
		int   floor_mass;                          // ���������ǰ�������
		int   prj_mass;                            // ���������ͶӰ��ĸ���
		int   floor_dif;                           // ��Ŀ���ƶ�������һ֡�ĸ߳�ͼ���

		int   root;                                // ��polar���������root��Ϣ

		int   accu_move;                           // ��Ŀ���ۻ����ƶ���

		short his_cx[MAX_TARGET_HISTORY];          // ��¼��Ŀ�����ʷλ��
		short his_cy[MAX_TARGET_HISTORY];          // ��¼��Ŀ�����ʷλ��
		clock_t his_tm[MAX_TARGET_HISTORY];        // ��¼��Ŀ�����ʷλ�õ�ʱ��
		short cur_his;                             // ��ʷλ��ָ��

		short wander_cx[MAX_WANDER_HISTORY];       // ��¼�����ǻ�������ͣ����λ��
		short wander_cy[MAX_WANDER_HISTORY];       // ��¼�����ǻ�������ͣ����λ��

		char  type;                                // �����������
		short status;                              // ����������״̬
		short is_evt;                              // �������Ƿ��¼�
		short is_evted;                            // �������Ƿ��Ѿ������¼�
		char  in_region;                           // ��Ǹ����崦����Щ������

		int   normal_cnt;                          // ��Ŀ�걻�ж�Ϊ�����Ĵ���
		int   float_cnt;                           // ��Ŀ�걻�ж�Ϊ�����Ĵ���
		int   floor_cnt;                           // ��Ŀ�걻�ж�Ϊƽ������Ĵ���
		int   people_cnt;                          // ��Ŀ�걻�ж�Ϊ����Ĵ���

		time_t tm_det;                             // ��¼Ŀ�걻������ʱ��
		time_t tm_lie;                             // ��¼Ŀ�꿪ʼ���ص�ʱ��
		time_t tm_still;                           // ��¼Ŀ����뾲ֹ״̬��ʱ��
		time_t tm_down;                            // ��¼Ŀ������¶�״̬��ʱ��
		time_t tm_follow;                          // ��¼Ŀ�����follow״̬��ʱ��
		time_t tm_run;                             // ��¼Ŀ�걨���ܲ���ʱ�̵�ʱ��

		int   move_cnt;                            // ��¼�����˶��˶���֡
		int   wander_cnt;                          // ��¼Ŀ���ǻ��˶��ٴΣ���ν�ǻ�����ָһ�����˶��£�ͣ��һ�ᣬ�ٴ��˶������Ƕ��
		int   fight_cnt;                           // ��¼Ŀ���ڴ��
		int   last_fight_det_cnt;                  // ��¼�ϴ�fight_cnt����ʱ��det_cnt

		short owner;                               // ��¼Ŀ���owner
		short last_owner;                          // ��¼Ŀ�����һ��owner
		short owner_dis;                           // ��¼Ŀ����owner�ľ���
		short owner_cnt;                           // ��¼Ŀ���owner�仯�˶��ٴ�
	} DepObject;

	typedef struct DepRootObject_{
		Rect rect_world;          // ���������λ��
		int  max_height;          // ��ߵĸ߶�
	} DepRootObject;

	typedef struct Depth_{
		// data about the frames---------------------------------
		unsigned short* pFrameDepth;
		unsigned char * pFrameGrey;
		unsigned char * pFrameRGB;
		unsigned char * pFrameGrad;
		short width, height;
		
		// data about the tracked objects----------------------------
		DepObject * pObjs;
		int n_objs;
		DepObject * pFinalObjs;            // ���ٵ��������Ϣ
		int n_final_objs;
		int cur_status;                    // ��ǰ��ģ��״̬
		int cam_evt;                       // ������ͷ��صı�����Ϣ
		
		// generated data===================================================================
		time_t cur_tm;
		// �����������---------------------------------
		unsigned short * pDepBuf[DEP_BUF_SIZE];    // ������֡��ʷ���ͼ�������޳�����
		int cur_dep_buf;                  // ��ǰ�Ļ���ָ��

		// ͼ���������-------------------------------------
		int acutance;                     // ͼ������
		int avg_grey[MAX_GREY_HISTORY];   // ͼ���ƽ���Ҷ�
		int cur_avg_grey;                 // ��¼��ǰ֡ƽ���Ҷȵ�ָ��
		int his_DFG[MAX_GREY_HISTORY];    // ��¼ͼ��ı궨��Ϣ
		int cur_dfg;                      // ��¼��ǰ֡�ı궨��Ϣָ��
		int avg_depth;                    // ƽ�����ֵ
		int dep_valid_cnt;                // ��Ч���ֵ����
		int up_pt_cnt;                    // ���ϵ�ĸ���
		int dfg_dif_percent;              // ���ǰ�����������ֵ��128�ֱȣ�

		int cam_fatal_cnt;                // �ж�����ͷ�����쳣�ļ�����
		int cam_ill_cnt;                  // �ж�����ͷ�����쳣�ļ�����
		int cam_slash_cnt;                // �ж�˲ʱ�쳣�ļ�����
		int cam_sun_cnt;                  // �����ǿ�ļ�����
		int cam_nofloor_cnt;              // �޷��ҵ��㹻�����ļ�����
		int cam_reinit_cnt;               // ��⵽˲��ʱ��󣬻��������³�ʼ�������ڼ�����ͷ�궨��Ϣ�ظ�����ȡ�����³�ʼ��

		// ����ͷ�궨----------------------------------------
		ImgPlaneAttr * pImgPlaneAttr;     // ��¼Ѱ��ƽ������е����ص�����
		Plane * pPlanes;                  // ��¼Ѱ�ҵ���ƽ��
		int plane_cnt;                    // ��¼ƽ��ĸ���
		float * x;                        // ��С������ƽ���x����
		float * y;                        // ��С������ƽ���y����
		float * z;                        // ��С������ƽ���z����
		float plane[4];                   // ��С������ƽ��õ���ƽ��, ax+by+cz=d
		float VD[4];                      // ����ó��ķ�����������߶�����
		float invX[3];                    // ����������ϵ�����������ϵ��任��ϵ����X����
		float invY[3];                    // ����������ϵ�����������ϵ��任��ϵ����Y����
		float invZ[3];                    // ����������ϵ�����������ϵ��任��ϵ����Z����
		int * pXCoeff;                    // ������������ϵ�����X������ұ�
		int * pYCoeff;                    // ������������ϵ�����X������ұ�
		int * pZCoeff;                    // ������������ϵ�����X������ұ�
		int   cam_height;                 // ����ͷ�ĸ߶�
		int   min_height;                 // ���������������͵�ĸ߶�

		// Ŀ����������-----------------------------------------------------
		FloorAttr * pFloorAttr;           // ��Ǹ���ͼ�������Ϣ
		FloorAttr * pFloorAttrLast;       // ��¼��һ֡�ĸ���ͼ�������Ϣ
		long long * pFloorHeightInte;     // �߳�ͼ�Ļ����ͱ�ʾ
		int * pFloorFGInte;               // �߳�ͼ��ǰ��������ͱ�ʾ
		int * pFloorPrjInte;              // �߳�ͼ��ͶӰ������Ļ����ͱ�ʾ
		int floor_attr_width;             // ����ͼ�����ݿ��
		int floor_attr_height;            // ����ͼ�����ݸ߶�
		int center_radius;                // �����ֲ��ߵ�����������
		int round_radiius;                // �����ֲ��ߵ���ܱ������
		int cr_stride;                    // �����ֲ��ߵ�Ĳ���
		ImgAttr * pImgAttr;               // ͼ����ص�����
		FloorTarget * pTargets;           // �Ӹ߶�ͼ�ϼ�����Ŀ�꼯��
		int floor_targets_cnt;            // �Ӹ߶�ͼ�ϼ�����Ŀ�����
		PolarTarget * pPolarTargets;      // �ֲ���ֵ�������Ŀ�꼯��
		PolarQueue  * pPolarQueue;        // ���ݾֲ���ֵ����Ŀ�����ǰ������ɢ�õĶ���
		int polar_targets_cnt;            // �ֲ���ֵ�������Ŀ�����
		RefineTarget * pImgTargets;       // ��ͼ���ϼ�����Ŀ�꼯��
		RefineTarget * pRefineTargets;    // �������Ŀ�꼯��
		int refine_targets_cnt;           // �������Ŀ�����
		short * edge_up;                  // ���ͼ�ϱ�Ե��λ��
		short * edge_left;                // ���ͼ���Ե��λ��
		short * edge_right;               // ���ͼ�ұ�Ե��λ��
		short * edge_bottom;              // ���ͼ�±�Ե��λ��
		char  * track_tag;                // ���ÿ������Ŀ���Ƿ񱻳ɹ�������

		// ǰ��������-------------------------------------------------------------
		short frame_cnt;                  // ��ǰ�����֡����ֻ��¼��10000
		BkgModel * pModel;                // ����ģ��
		int   plus_percent;               // ��ѧ���������ٷֱ�(ʵ������128�ֱ�)
		short * map;                      // ����floodfillʱʹ�õ�IDӳ���
		short * obj_map;                  // ����Ŀ��ϲ�ʱʹ�õ�IDӳ���
		BGTarget * bg_targets;            // ����Ŀ��
		int bg_targets_cnt;               // ����Ŀ��ĸ���

		// ���Ƶ���������---------------------------------------------------------
		short center_u,center_v;          // ����ԭ������Ļ�ϵ�����
		short xdirect_u,xdirect_v;        // ����X�᷽������Ļ�ϵ�����

		// �����йص�����ͷ�¼��ж�--------------------------------------------------
		int   people_cnt;                 // �����ڵ�������
		int   no_people_cnt;              // ���������˵�֡��
		int   follow_people_cnt;          // ��¼��һ֡�ж����˴�����Ҫ��follow�¼��ĵ�
		int   fight_people_cnt;           // ��¼��һ֡�ж����˴�����Ҫ��fight�¼��ĵ�

		// ��ֹ���屨������----------------------------------------------------------
		ObjectTrack obj_track[MAX_OBJ_TRACK]; // ���ٵ����������Ϣ
		unsigned char * pFrameBG;             // �ӱ���ģ���еõ��ı���ͼ

	  // settings--------------------------------------------------
		int max_blobs;       // ��Ҫ�����˶��������
		int min_blob_size;   // ͼ����ʱ����С��Ŀ���С��С�������С�Ľ�����
		int min_floor_blob_size; // ������ʱ����С��Ŀ���С��С�������С�Ľ�����
		int odd_pt_th;           // �ж�ĳ��������ֵ�ǲ�����������ֵ����ֵ

		int plane_thresh;    // �ж��ĸ�����һ��ƽ�����ֵ������ֵ��1/1000
		int floor_y_base;    // ����ͼy����Ļ�׼
		int floor_unit;      // ����ͼ����ĵ�λ
		int floor_x_span;    // ����ͼx����Ŀ��
		int floor_y_span;    // ����ͼy����Ŀ��
		int floor_hthresh;   // ĳ��ĸ߶ȸ��ڸ�ֵ����Ϊ��ǰ��
		int floor_wall_th;   // ĳ�����������ϰ�����ǽ��128�ֱ���ֵ�����ڸ���ֵ��Ϊ��ǽ

		int aim_head_r;      // ���ֲ��ߵ�����Ŀ��
		int aim_body_r;      // ���ֲ��ߵ���ܱ߿��
		int head_body_th;    // ������ƽ���߶ȱ��ܱ�ƽ���߶ȸ�head_body_th����ʱһ���ٷֱȣ����϶�ΪǱ��ͷ��
		int head_body_stride;// �����ֲ��ߵ�ʱ����������

		int vd_learn_rate;   // VD��ѧϰ�ʣ���ǧ�ֱȱ�ʾ
		int cam_height_adj;  // ����ͷ��ʵ�İ�װ�߶�

		int active_cnt_max;  // ÿ����active_cnt�����ֵ
		int ref_support_th;  // ÿ��Ŀ��֧�ŵ��128�ֱ���ֵ
		int ref_people_th;   // �ж�����Ϊ�������ֵ���߶ȣ�
		int ref_rid_th;      // �����rid����ռ������128�ֱȸ��ڸ�ֵ������Ϊ������Ӧ�����Ӧ��Floor����Ϊ��
		int ref_rid_merge_th;// �����������ͼ���������²��С�ڸ�ֵ�����������ص�����ϲ�������
		int ref_merge_th_uv; // �������������ڵ�ͼ�������InnerDif���ڸ�ֵ����ϲ�����������
		int ref_merge_th_xy; // �������������ڵ����������InnerDif���ڸ�ֵ����ϲ�����������

		int pt_neighbor_th;  // ����������������������ֵ
		int pt_dep_dif_th;   // �õ��뱳��������ƽ������С�ڸ�ֵ������Ϊ��Ա���������㹻�����ܹ�����ճ����
		
		int samples_per_pix; // ÿ�����ص���������
		int dif_thresh_grey; // ��ѧǰ���Ҷ��뱳���������ֵ
		int dif_thresh_grad; // ��ѧǰ���ݶ���ֵ
		int dif_thresh_shad; // ��ѧǰ����Ӱ�ж���ֵ������ֵ��dif_thresh_grey��128�ֱȱ�ʾ����ֵӦ����128
		int dif_thresh_hue;  // ��ѧǰ��ɫ���뱳���������ֵ
		int dif_thresh_dep;  // ���ǰ������뱳���������ֵ
		int bg_cnt_thresh;   // ����ٸ�������ƥ�����Ϊ�Ǳ���
		int n_subsampling;   // �ж��Ǳ����󣬸��½���������RGB�ĸ���Ϊ1/n_subsampling
		int n_subsampling_d; // �ж��Ǳ����󣬸��½�����������ȵĸ���Ϊ1/n_subsampling_d
		int neighbor_span;   // �����ھӵ������С

		int plus_step;       // ������Ĳ���
		int plus_cnt;        // ������������̽��Ĵ���
		int try_step;        // DepthTryForeGroundʹ�õ�̽�ⲽ��

		int cam_h_low;       // ����ͷ�߶ȵ�����
		int cam_h_high;      // ����ͷ�߶ȵ�����
		int fu,fv;           // ͼ������ϵ������ͷ����ϵ��ת����ϵ��

		int track_uv_th;     // Ŀ�����ʱ��������Ŀ���ͼ����ξ���С�ڸ�ֵʱ��Ϊ��ͬһĿ��
		int track_xy_th;     // Ŀ�����ʱ��������Ŀ���������ξ���С�ڸ�ֵʱ��Ϊ��ͬһĿ��
		int track_height_th; // Ŀ�����ʱ��Ҫ�������Ŀ��߶���ԭ�߶�֮��ܸ��ڸ�ֵ����ֵΪ128�ֱ�
		int track_inner_th;  // Ŀ�����ʱ��Ҫ�������Ŀ��inner���������ڸ�ֵ
		int track_max_life;  // Ŀ�����ʱ��life�����ֵ
		int track_still_th;  // Ŀ�����ʱ���ƶ�����С�ڸ�ֵ��Ϊ����û���˶�
		int track_wander_th; // Ŀ���뿪ԭ�ض�Զ������Ϊ���н���WANDER״̬��Ǳ��
		int track_still_cnt_th; // Ŀ�����still״̬����֡���Ϳ��Խ�����ΪWANDER��һվ��
		int track_run_th;    // Ŀ�����ʱ���ٶȴ��ڸ�ֵ��Ϊ�������ܲ�������/��
		int track_near_th;   // Ŀ�����ʱ�������������ڸ�ֵ��������Ŀ�����̫��
		int track_owner_th;  // Ŀ�����ʱ������������������ڸ�ֵ��������Ŀ�����̫��
		int track_owner_cnt_th; // �������owner�Ĵ������ڸ�ֵ������still_cnt���㣬�������ı�������------------------------
		int track_down_th;   // Ŀ��߶Ƚ��ͺ�����ռ����Ŀ�/����������վ��ʱ��/���ı��������϶����������¶ף�128�ֱ�

		int floor_pt_th;        // ��һ����������ǰ����ٷֱȵ��ڸ���ֵʱ����Ϊ��ƽ����
		int people_veri_th;     // ��һ�����������ٶȸ��ڸ�ֵ�󣬲��϶�������
		int people_area_th;     // ��һ��Ŀ���ʵ��������ڸ�ֵ�����п����϶�Ϊ��
		int people_veri_cnt_th; // ��һ��Ŀ�걻�����Ĵ������ڸ�ֵ�����п����϶�Ϊ��

		int people_run_th;      // �ж�һ�������ܲ�����ֵ����������һ�������ܲ���ô�����ڲ����ٱ��ܲ�
		int people_lie_th;      // �ж�һ������ˤ������ֵ���߶�128�ֱ�
		int people_lie_cnt_th;  // �ж�һ������ˤ������ֵ������
		int people_lie_long_th; // �ж�һ���˳�ʱ�䵹�ص���ֵ������
		int people_stay_th;     // �ж�һ������/������������ֵ������
		int people_wander_dis_th;//�ж�һ�����ǻ�����ֵ�����ƶ����룬����
		int people_follow_th;   // �ж����������໥���ٵ���ֵ������
		int people_fight_th;    // �ж����������ܵ���ֵ
		int people_fight_det_th;// �ж��������ܣ�������κϲ���ļ��������ڸ�ֵ������Ϊ���
		int people_down_th;     // �ж�һ�����¶׵���ֵ���߶�128�ֱ�
		int people_down_cnt_th; // �ж�һ�����¶׵���ֵ������
		int object_still_th;    // �ж�һ��������������ֵ������

		int obj_track_th;       // �ж��¼�����������Ժϲ����Ѹ����������ֵ

		int frame_ignore;    // �ڳ�ʼ��ǰ�����Ⱥ��Ե�֡��
		int frame_init;      // �ڳ�ʼ��ʱ����������ȹ�����Ҫ�����֡��

		int light_up_th;     // ���ȹ��ߵ���ֵ
		int light_down_th;   // ���ȹ��͵���ֵ
		int light_thresh;    // ���ȱ仯����ֵ
		int blur_thresh;     // ����ͷģ������ֵ����ȹ��ͣ����ڸ�ֵ��Ϊģ��
		int dep_valid_thresh;// ��Ч���ֵ������ֵ��128�ֱ�
		int cover_thresh;    // �ж�����ͷ�ڵ�����ֵ����ƽ�����ֵ���ڸ�ֵ����Ϊ�ڵ�
		int floor_cnt_thresh;// �ж������������͵���ֵ,������ǧ�ֱ�
		int cam_move_thresh;  // �ж�����ͷ�ƶ�����ֵ,128�ֱ�
		int door_fg_thresh;  // �ж����ŵ���ֵ��ǰ�����128�ֱ�
		int cover_span;      // �ж�����ͷ�ڵ���ȫ��ʱ����Ҫ�ۻ���֡��

		int region_in_vio_th; // ���뵼������Υ��ʱ��ǰ����������֡û�䣬����Ϊ��ǿ�н���
		int region_pre_cnt_th;// �����pre_people_cntά�ֶ���֡���Ὣ���Ϊcur_cnt_frames
		int region_fbuf_th;   // �����ڵ���Ա������people_down����֡�󣬱����������¼�

		int ct_max_life;     // ��¼��ճ�����������ֵ

		// region settings---------------------------------------------------
		Region valid_region;             // ��Ч����
		Region regions[REGION_MAX_CNT];  // �������
		
		// ��������----------------------------------------------------------
		int frame_bytes;     // һ��RGB֡��Ҫ���ڴ�
		int frame_len;       // һ��RGB֡�ĵ���
		int frame_line_bytes;// һ��RGB֡�е��ڴ�ռ����

		// ��ʽת����ص�---------------------------------------------------
		unsigned short tab_r[256];
		unsigned short tab_g[256];
		unsigned short tab_b[256];

	} Depth;
	
// �ⲿ�ӿ�===============================================================================================
//��ʼ��Depth���ݽṹ
// pDep: ���ģ������Ҫ�����ݽṹ
// width��height��ͼ��Ŀ�Ⱥ͸߶�
// path:Depthģ�������ļ�·��
// �ɹ�ʱ����true�����򷵻�false������falseʱ��Ҫ��������ļ���·�����Լ����õĲ����Ƿ����
bool DepthInit( Depth* pDep, int width, int height, char* path );

// �ͷ�Depth���ݽṹ����Դ
// pDep: Depth�����ָ��
void DepthUninit( Depth* pDep );

// �����/ɫ��ͼ����д���
// pDep: ���ģ������Ҫ�����ݽṹ
// pRGB: RGB��ʽ����ͼ��
// pDepth:��ȸ�ʽ����ͼ��
int DepthProcess( Depth * pDep, unsigned char * pRGB, unsigned short * pDepth );

int DepthChangeStatus( Depth * pDep, int status );

// �ڲ�ʹ��===============================================================================================

// ɾ�����ͼ�е��쳣��-------------------------------------------------------------------------
void FilterOutDepth( Depth * pDep, unsigned short * pDepth );

// ���ͼ���������ڵ���-------------------------------------------------------------------------
void CheckCamQuality( Depth * pDep );
int  AcutanceAndGrey( Depth * pqua, unsigned char * pFrameGrey ); // ������ȣ�����ƽ���Ҷ�

// Ѱ�ҵ���ͱ궨���������---------------------------------------------------------------------
void  SearchFloor( Depth* pDep );
float DecideCamParamMin2( Depth * pDep, float * VD );
void  RefineFloor( Depth * pDep );
void  ApplyCamParam( Depth * pDep, float * VD );
void  CalcAttr( Depth* pDep );

void SearchPlane( Depth * pDep );
bool CalcDirection( float * CP1, float * CP2, float * CP3, float * CP4, float * VD, float thresh );
void CalcPT( float pt[3], unsigned short dep, int u, int v, int fu, int fv );

// ��ȴ�������---------------------------------------------------------------------------------
bool DetectTargets( Depth * pDep, short * map );
int  DetectPolarTargets( Depth * pDepth, int region_idx );
void RefinePolarTargets( Depth * pDepth );
void ReverseMap( Depth * pDep );
bool RefineTargets( Depth * pDep, short * map );
void TrackTargets( Depth * pDep );

// ��Ϲ�ѧǰ�����-----------------------------------------------------------------------------
void DepthDetectMotion(Depth* pDep);
void DepthBkgInit(Depth* pDep,int idx);
int  DepthTryForeGround( Depth* pDep, int plus );
int  DepthDetForeGround( Depth* pDep, int plus );
void DepthBkgUpdate(Depth* pDep);
void DepthRefineFG( Depth* pDep );

// �Ѿ�������ƽ���������-----------------------------------------------------------------------
int  CheckTrack( Depth * pDep, int idx );
void HandleTrack( Depth * pDep );
int  Different( Depth * pDep, unsigned char * l, unsigned char * r, Rect rrect );
void GenBG( Depth * pDep );

// ���ߺ���-------------------------------------------------------------------------------------
void DepthRGB2Grey( Depth * pDep, unsigned char* pDFrameGrey, unsigned char * pSFrame );
void DepthGenGrad( Depth * pDep );
int  DepthRectDif( Rect l, Rect r, Rect * pcommon );  // ��������������غ϶ȣ�0~100
int  DepthInnerDif( Rect l, Rect r );  // ��������������غ϶ȣ�0~100������Զ�����С��Ϊ��ĸ
int  DepthOuterDif( Rect l, Rect r );   // ��������������غ϶ȣ�0~100������Զ�����С��Ϊ��ĸ
int  RectDistance( Rect l, Rect r );    // �������������ľ��룬���ĸ���֮��������Ƕ�Ϊ׼
int  UpdateRegion( Depth* pDep );       // �������úõ������x��y���꣬����ɹ�����1�����򷵻�-1
Rect MergeRect( Rect l, Rect r );      // �ϲ�����rect
Rect AndRect( Rect l, Rect r );        // ȡ����rect��������
int  CalcSpeed( Rect pre, Rect now, short * spx, short * spy );    // ����������˶��ٶȣ���֪������ռ����
void HandleRegionPeopleCnt( Depth * pDep, Region * pregion, int people_cnt );    // ����ĳ�������������صĶ���
void CalIntegralFloor( long long* pInte, int * pFloorFGInte, int * pFloorPrjInte, FloorAttr* pFloor, int width, int height ); // ����߳�ͼ�Ļ���ͼ
void World2UV( Depth * pDep, int * u, int * v, int x, int y, int h );        // ��������ϵת��Ϊͼ������ϵ
bool IsCross(Point pt1, Point pt2, Point line_s, Point line_e, Point dir);
int  FloorDif( Depth * pDep, Rect curr, Rect last );
int  IsInRect( int u, int v, Rect rct );

// ������---------------------------------------------------------------------------------------
void SaveID( char * path, FloorAttr * pAttr, short * map, int width, int height );
void SaveIDImg( char * path, ImgAttr * pAttr, short * map, int width, int height );

#ifdef __cplusplus
}
#endif

#endif
