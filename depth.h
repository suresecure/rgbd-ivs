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

#define DEPTH_STATUS_INIT      0   // 初始化状态
#define DEPTH_STATUS_NORM      1   // 正常工作状态
#define DEPTH_STATUS_FATAL     2   // 摄像头致命错误状态，这种状态下不进行任何分析，包括过亮、过暗、遮挡、模糊
#define DEPTH_STATUS_ILL       3   // 非致命错误状态，主要包括有效深度值过少，没有看到地面
#define DEPTH_STATUS_ERROR     4   // 错误状态，模块发生了错误

#define REGION_MAX_CNT         8  // 本模块最多能够设置的区域个数
#define REGION_MAX_PTS         16 // 每个多边形区域包含的最多点数

#define REGION_STATUS_UNSET      0  // 无效状态
#define REGION_STATUS_NORMAL     1  // 正常状态
#define REGION_STATUS_VIOLATE    2  // 违规状态

#define REGION_TYPE_NORMAL      0  // 无属性区，不报任何事件
#define REGION_TYPE_METER       1  // 防护区，仅允许区域内人数在一定范围之内，区域事件：人数过多、人数过少、人数回复正常
#define REGION_TYPE_CROSS       2  // 拌线区，用于指明某个拌线的“线”和方向，区域事件：人员穿过
#define REGION_TYPE_VALID       3  // 有效区，目标要报警必须在某个有效区内，区域事件：人员进入、人员离开
#define REGION_TYPE_DOOR        4  // 门监控区，监控某个门的打开、关闭
#define REGION_TYPE_VIO         5  // very important object划定区，以防重要物品丢失

#define EVT_LIEDOWN       1   // 目标倒地事件
#define EVT_RUNNING       2   // 目标跑步事件
#define EVT_LIEDOWN_LONG  4   // 目标长时间到底事件（疑似晕倒）
#define EVT_FIGHT         8   // 目标打架事件
#define EVT_STAY          16  // 目标滞留事件（人物留在里面过长）
#define EVT_WANDER        32  // 目标徘徊事件（人物不仅留下，而且来回运动）
#define EVT_STILL         64  // 物品遗留事件（物体保持不动）
#define EVT_DOWN          128 // 目标下蹲事件
#define EVT_PLANE_STILL   256 // 粘贴物滞留事件（平面物体，完全由rgb检测得到的物体）
#define EVT_SAMPLE_FRAME  512 // 样本采集事件

#define EVT_CAM_COVER_BLUR_DARK_BRIGHT      1   // 摄像头致命异常：遮挡、模糊、过亮、过暗
#define EVT_CAM_MOVED_LIGHT_UP_DOWN         2   // 摄像头瞬时异常：移动、亮度突变
#define EVT_CAM_NOFLOOR                     4   // 摄像头没有看到地面
#define EVT_CAM_DEP_VALID_SMALL             8   // 有效深度值过少
#define EVT_CAM_SOMEFOLLOW                  16  // 尾随事件，作为摄像头事件来报
#define EVT_CAM_FIGHT                       32  // 打架事件，也作为摄像头来报

#define EVT_REGION_CNT_VIO_UP    1   // 区域人数违规事件(人数过多，指连续多人进入）
#define EVT_REGION_CNT_VIO_DOWN  2   // 区域人数违规事件(人数过少）
#define EVT_REGION_OBJ_IN        4   // 区域有人进入事件
#define EVT_REGION_OBJ_OUT       8   // 区域有人离开事件
#define EVT_REGION_IN_VIO        16  // 区域有人违规进入（强行推入导致人数过多，指人数固定一段时间后，又有人进入导致人数过多）
#define EVT_REGION_CNT_NORMAL    32  // 区域人数恢复正常事件
#define EVT_REGION_CROSS         64  // 区域有人穿过（实际是拌线有人穿过）
#define EVT_CAM_SINGLE_OPEN      128 // 区域单人加钞，场景只有一人，且钞箱门打开或被遮挡

#define PT_TYPE_INVALID  0       // 标记该点是无效（背景）点
#define PT_TYPE_DVALID   1       // 标记该点的深度值是有效的
#define PT_TYPE_FLOOR    2       // 标记该点是地面点
#define PT_TYPE_FG       4       // 标记该点是前景点
#define PT_TYPE_FG_DEP   8       // 标记该点为深度检测前景
#define PT_TYPE_FG_RGB   16      // 标记该点为彩色检测前景
#define PT_TYPE_FG_GRAD  32      // 标记该点为梯度检测前景点，是梯度前景点的点一定也是彩色检测前景点
#define PT_TYPE_SHADOW   64      // 标记该点是阴影点
#define PT_TYPE_SUPPORT  128     // 标记该点是支撑点
#define PT_TYPE_FG2BG    256     // 标记该点是前景转背景
#define PT_TYPE_FG_EXT   512     // 标记该点是经过Refine后的前景

#define TARGET_TYPE_NORMAL 0     // 刚刚检测出的无类型物体
#define TARGET_TYPE_PEOPLE 1     // 判定为人员的物体
#define TARGET_TYPE_FLOOR  2     // 完全由光学检测出的物体，不一定在地面上
#define TARGET_TYPE_WALL   3     // 判定为墙壁
#define TARGET_TYPE_FLOAT  4     // 支撑不足的物体（悬空物体）

#define TARGET_STATUS_LIEDOWN      1   // 倒地状态
#define TARGET_STATUS_MOVE         2   // 移动状态
#define TARGET_STATUS_FOLLOW       4   // 尾随状态
#define TARGET_STATUS_STAY_STILL   16  // 滞留状态（人物/物体）
#define TARGET_STATUS_DOWN         64  // 下蹲状态

#define PLANE_TYPE_NO    0  //不是平面点
#define PLANE_TYPE_UP    1  //方向朝上的平面点
#define PLANE_TYPE_HZ    2  //方向水平的平面点
#define PLANE_TYPE_OTHER 3  //方向不规则的平面点

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
		short min_height;         // 投射到该位置的最低高度
		short min_u,min_v;        // 最低点投影的图像坐标
		short max_height;         // 投射到该位置的最高高度
		short max_u,max_v;        // 最高点投影的图像坐标
		short prj_cnt;            // 投影到该位置的图像点的个数

		short id;                 // 物体检测时该点的ID号
		short polar_id;           // polar物体检测时该点的ID号
		short polar_dis;          // polar物体晕染到该点花费的轮数
	} FloorAttr;

	typedef struct FloorTarget_{
		short max_height;         // 该目标最高点高度
		short x,y;                // 该目标的位置
		Rect  rect;               // 该目标所占的图上区域
		short id;                 // 该目标的ID
		int   mass;               // 该目标所包含的点个数
		int   floor_mass;         // 该目标所包含的紧贴地面的点的个数

		short children_cnt;       // 该目标的子目标个数
	} FloorTarget;

	typedef struct PolarTarget_{
		short max_height;         // 该目标最高点高度
		short x,y;                // 该目标的位置

		short avg_height;         // 该目标中心区的平均高度
		Rect  rect;               // 该目标所占的投影图的区域
		Rect  rect_world;         // 该目标所占的实际区域
		Rect  rect_image;         // 该目标所占的图上区域

		short root;               // 该目标所在的大目标

		int   mass;               // 该目标所包含的点个数
		int   floor_mass;         // 该目标所包含的紧贴地面的点的个数
		int   img_mass;           // 该目标的图上点的个数
		int   support_mass;       // 该目标的支撑点

		// 从RefineTarget拷贝的信息
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
		short max_height;         // 该目标最高点高度
		int   x,y;                // 该目标的实际位置（最高点）
		Rect  rect;               // 该目标所占的图上区域
		Rect  rect_world;         // 该目标所占的实际区域
		short id;                 // 该目标分配的ID号
		short children_cnt;       // 该目标内的Polar目标的个数

		int   mass;               // 该目标所包含的点个数
		int   active_mass;        // 该目标包含的活跃点个数
		int   dep_mass;           // 该目标包含的由深度信息检测出的点的个数
		int   grad_mass;          // 该目标包含的由梯度信息检测出的点的个数
		int   support_mass;       // 该目标包含的支撑点个数
		int   floor_mass;         // 该目标包含的地面点个数
		int   prj_mass;           // 该目标包含的投影点个数
		char* low_pt;             // 该目标的最下方点的相关信息
		int*  rid_cnt;            // 该目标的rid映射富集度
	} RefineTarget;

	typedef struct ImgAttr_{
		short depth;               // 该点的深度值
		short x,y;                 // 该点的x,y坐标（地面坐标系）
		short height;              // 该点的实际高度
		short type;                // 该点的类型
		short active_cnt;          // 活跃计数，如果该点被某个点覆盖，则该点进入活跃状态
		short plane_type;          // 该点的平面类型
		short is_in_vir;           // 是否在VIR中（very important region）
		short avg_bg_dep;          // 该点的背景点平均深度
		short bg_height;           // 记录该点出背景的高度

		short id;                  // 该点的ID
		short rid;                 // 反映射得到的ID
		short polar_id;            // 反映射得到的polar_id
		int   offset;              // 该点映射到平面图的坐标
	} ImgAttr;

	typedef struct BGTarget_{
		Rect rect;                 // 该目标的图上区域
		int mass;                  // 该目标的总点数
	} BGTarget;

	typedef struct ImgPlaneAttr_{
		float x,y,z;               // 该点的x,y,z坐标
		float height;              // 高度
		short plane_type;          // 该点的平面类型
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

		short u[REGION_MAX_PTS]; // 标记该区域的四个点的u坐标
		short v[REGION_MAX_PTS]; // 标记该区域的四个点的v坐标
		short x[REGION_MAX_PTS]; // 标记该区域的四个点的x坐标
		short y[REGION_MAX_PTS]; // 标记该区域的四个点的y坐标
		short n_pt;              // 该多边形区域包含的点数

		short type;              // 该多边形的类型

		short people_up;         // 标记该区域人员的最大数量
		short people_down;       // 标记该区域人员的最小数量

		char  status;            // 标记该区域的状态
		short is_evt;            // 标记该区域是否要报事件，这是一个mask型的变量

		char  cur_people_cnt;    // 标记该区域的当前人数
		short cur_cnt_frames;    // 标记该区维持现在的人数的帧数
		char  pre_people_cnt;    // 区域内人数正要变成多少
		short pre_cnt_frames;    // 标记pre_people_cnt维持当前值的帧数

		int   dfg_cnt;           // 区域内深度前景点的个数
		short big_fg_frames;     // 区域前景点过多的帧数
	} Region;

	typedef struct _ObjectTrack{
		Rect rect;
		unsigned char * bg;   // 当前物体的背景图片
		unsigned char * fg;   // 当前物体的前景图片
		int life;        // 当前物体跟踪的生命值

		// 临时数据
		int fg_score;    // 最后一次合并进来的前景差异
		int bg_score;    // 最后一次合并进来的背景差异
	} ObjectTrack;

	// 关于检测到的物体需要记录的属性---------------------------------------------
	typedef struct DepObject_{
		Rect  region;                              // 包裹该物体的外接矩形框
		Rect  region_world;                        // 地面上所占区域
		Rect  lregion;                             // 该目标上一帧的位置
		short speed_x,speed_y;                     // 地面上的速度
		short max_move_cnt;                        // 记录目标曾经达到的最大速度
		int   speed;                               // 记录目标当前的速度

		Rect  region_prj;                          // 记录该目标高程图的位置
		Rect  region_prj_last;                     // 记录该目标上一次的高程图位置
		Point region_floor[4];                     // 地面上的投影在rgb图上的区域
		Point region_head[4];                      // 物体的头顶在rgb图上的区域

		short ori_cx, ori_cy;                      // 目标的初始位置
		int   height;                              // 物体当前高度
		int   max_height;                          // 目标历史高度最大值
		int   cx, cy;                              // 最高点的位置
		int   cwidth,cheight;                      // 最高点时目标的宽度和高度
		short life;                                // 物体的生命值
		int   det_cnt;                             // 物体被跟踪的帧数

		int   mass;                                // 该物体所包含的前景点个数
		int   support_mass;                        // 该物体支撑点个数
		int   dep_mass;                            // 该物体的深度前景点个数
		int   floor_mass;                          // 该物体地面前景点个数
		int   prj_mass;                            // 该物体包含投影点的个数
		int   floor_dif;                           // 该目标移动后与上一帧的高程图差别

		int   root;                                // 由polar物体带来的root信息

		int   accu_move;                           // 该目标累积的移动量

		short his_cx[MAX_TARGET_HISTORY];          // 记录该目标的历史位置
		short his_cy[MAX_TARGET_HISTORY];          // 记录该目标的历史位置
		clock_t his_tm[MAX_TARGET_HISTORY];        // 记录该目标的历史位置的时间
		short cur_his;                             // 历史位置指针

		short wander_cx[MAX_WANDER_HISTORY];       // 记录人物徘徊过程中停留的位置
		short wander_cy[MAX_WANDER_HISTORY];       // 记录人物徘徊过程中停留的位置

		char  type;                                // 该物体的类型
		short status;                              // 该物体所处状态
		short is_evt;                              // 该物体是否报事件
		short is_evted;                            // 该物体是否已经报过事件
		char  in_region;                           // 标记该物体处于哪些区域内

		int   normal_cnt;                          // 该目标被判定为正常的次数
		int   float_cnt;                           // 该目标被判定为悬浮的次数
		int   floor_cnt;                           // 该目标被判定为平面物体的次数
		int   people_cnt;                          // 该目标被判定为人物的次数

		time_t tm_det;                             // 记录目标被检测出的时间
		time_t tm_lie;                             // 记录目标开始倒地的时间
		time_t tm_still;                           // 记录目标进入静止状态的时间
		time_t tm_down;                            // 记录目标进入下蹲状态的时间
		time_t tm_follow;                          // 记录目标进入follow状态的时间
		time_t tm_run;                             // 记录目标报“跑步”时刻的时间

		int   move_cnt;                            // 记录物体运动了多少帧
		int   wander_cnt;                          // 记录目标徘徊了多少次，所谓徘徊，是指一个人运动下，停留一会，再次运动，如是多次
		int   fight_cnt;                           // 记录目标在打架
		int   last_fight_det_cnt;                  // 记录上次fight_cnt递增时的det_cnt

		short owner;                               // 记录目标的owner
		short last_owner;                          // 记录目标的上一个owner
		short owner_dis;                           // 记录目标与owner的距离
		short owner_cnt;                           // 记录目标的owner变化了多少次
	} DepObject;

	typedef struct DepRootObject_{
		Rect rect_world;          // 物理世界的位置
		int  max_height;          // 最高的高度
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
		DepObject * pFinalObjs;            // 跟踪的物体的信息
		int n_final_objs;
		int cur_status;                    // 当前的模块状态
		int cam_evt;                       // 与摄像头相关的报警信息
		
		// generated data===================================================================
		time_t cur_tm;
		// 清除奇异数据---------------------------------
		unsigned short * pDepBuf[DEP_BUF_SIZE];    // 缓存三帧历史深度图，用于剔除误差点
		int cur_dep_buf;                  // 当前的缓冲指针

		// 图像质量诊断-------------------------------------
		int acutance;                     // 图像的锐度
		int avg_grey[MAX_GREY_HISTORY];   // 图像的平均灰度
		int cur_avg_grey;                 // 记录当前帧平均灰度的指针
		int his_DFG[MAX_GREY_HISTORY];    // 记录图像的标定信息
		int cur_dfg;                      // 记录当前帧的标定信息指针
		int avg_depth;                    // 平均深度值
		int dep_valid_cnt;                // 有效深度值个数
		int up_pt_cnt;                    // 向上点的个数
		int dfg_dif_percent;              // 深度前景点差异评估值（128分比）

		int cam_fatal_cnt;                // 判断摄像头致命异常的计数器
		int cam_ill_cnt;                  // 判断摄像头致命异常的计数器
		int cam_slash_cnt;                // 判断瞬时异常的计数器
		int cam_sun_cnt;                  // 阳光过强的计数器
		int cam_nofloor_cnt;              // 无法找到足够地面点的计数器
		int cam_reinit_cnt;               // 检测到瞬发时间后，会启动重新初始化，若期间摄像头标定信息回复，则取消重新初始化

		// 摄像头标定----------------------------------------
		ImgPlaneAttr * pImgPlaneAttr;     // 记录寻找平面过程中的像素点属性
		Plane * pPlanes;                  // 记录寻找到的平面
		int plane_cnt;                    // 记录平面的个数
		float * x;                        // 最小二乘求平面的x坐标
		float * y;                        // 最小二乘求平面的y坐标
		float * z;                        // 最小二乘求平面的z坐标
		float plane[4];                   // 最小二乘求平面得到的平面, ax+by+cz=d
		float VD[4];                      // 计算得出的法向量及相机高度向量
		float invX[3];                    // 由世界坐标系向摄像机坐标系逆变换的系数：X分量
		float invY[3];                    // 由世界坐标系向摄像机坐标系逆变换的系数：Y分量
		float invZ[3];                    // 由世界坐标系向摄像机坐标系逆变换的系数：Z分量
		int * pXCoeff;                    // 计算世界坐标系坐标的X方向查找表
		int * pYCoeff;                    // 计算世界坐标系坐标的X方向查找表
		int * pZCoeff;                    // 计算世界坐标系坐标的X方向查找表
		int   cam_height;                 // 摄像头的高度
		int   min_height;                 // 整个场景里面的最低点的高度

		// 目标检测跟踪相关-----------------------------------------------------
		FloorAttr * pFloorAttr;           // 标记俯视图的相关信息
		FloorAttr * pFloorAttrLast;       // 记录上一帧的俯视图的相关信息
		long long * pFloorHeightInte;     // 高程图的积分型表示
		int * pFloorFGInte;               // 高程图的前景点积分型表示
		int * pFloorPrjInte;              // 高程图的投影点个数的积分型表示
		int floor_attr_width;             // 俯视图的数据宽度
		int floor_attr_height;            // 俯视图的数据高度
		int center_radius;                // 搜索局部高点的中心区宽度
		int round_radiius;                // 搜索局部高点的周边区宽度
		int cr_stride;                    // 搜索局部高点的步长
		ImgAttr * pImgAttr;               // 图像相关的数据
		FloorTarget * pTargets;           // 从高度图上检测出的目标集合
		int floor_targets_cnt;            // 从高度图上检测出的目标个数
		PolarTarget * pPolarTargets;      // 局部极值点检测出的目标集合
		PolarQueue  * pPolarQueue;        // 依据局部极值点检测目标进行前景点扩散用的队列
		int polar_targets_cnt;            // 局部极值点检测出的目标个数
		RefineTarget * pImgTargets;       // 从图像上检测出的目标集合
		RefineTarget * pRefineTargets;    // 精化后的目标集合
		int refine_targets_cnt;           // 精化后的目标个数
		short * edge_up;                  // 深度图上边缘的位置
		short * edge_left;                // 深度图左边缘的位置
		short * edge_right;               // 深度图右边缘的位置
		short * edge_bottom;              // 深度图下边缘的位置
		char  * track_tag;                // 标记每个跟踪目标是否被成功跟踪了

		// 前景检测相关-------------------------------------------------------------
		short frame_cnt;                  // 当前处理的帧数，只记录到10000
		BkgModel * pModel;                // 背景模型
		int   plus_percent;               // 光学方面的增益百分比(实际上是128分比)
		short * map;                      // 进行floodfill时使用的ID映射表
		short * obj_map;                  // 进行目标合并时使用的ID映射表
		BGTarget * bg_targets;            // 背景目标
		int bg_targets_cnt;               // 背景目标的个数

		// 绘制地面标线相关---------------------------------------------------------
		short center_u,center_v;          // 地面原点在屏幕上的坐标
		short xdirect_u,xdirect_v;        // 地面X轴方向在屏幕上的坐标

		// 与人有关的摄像头事件判定--------------------------------------------------
		int   people_cnt;                 // 区域内的总人数
		int   no_people_cnt;              // 区域内无人的帧数
		int   follow_people_cnt;          // 记录上一帧有多少人处在需要报follow事件的点
		int   fight_people_cnt;           // 记录上一帧有多少人处在需要报fight事件的点

		// 静止物体报警缓存----------------------------------------------------------
		ObjectTrack obj_track[MAX_OBJ_TRACK]; // 跟踪的物体相关信息
		unsigned char * pFrameBG;             // 从背景模型中得到的背景图

	  // settings--------------------------------------------------
		int max_blobs;       // 需要检测的运动物体个数
		int min_blob_size;   // 图像检测时，最小的目标大小，小于这个大小的将忽略
		int min_floor_blob_size; // 地面检测时，最小的目标大小，小于这个大小的将忽略
		int odd_pt_th;           // 判定某个点的深度值是不正常的奇异值的阈值

		int plane_thresh;    // 判定四个点在一个平面的阈值，该阈值是1/1000
		int floor_y_base;    // 俯视图y坐标的基准
		int floor_unit;      // 俯视图坐标的单位
		int floor_x_span;    // 俯视图x方向的跨度
		int floor_y_span;    // 俯视图y方向的跨度
		int floor_hthresh;   // 某点的高度高于该值则认为是前景
		int floor_wall_th;   // 某个地面物体上包含的墙点128分比阈值，高于该阈值认为是墙

		int aim_head_r;      // 检测局部高点的中心宽度
		int aim_body_r;      // 检测局部高点的周边宽度
		int head_body_th;    // 若中心平均高度比周边平均高度高head_body_th，这时一个百分比，则认定为潜在头部
		int head_body_stride;// 搜索局部高点时的搜索步长

		int vd_learn_rate;   // VD的学习率，用千分比表示
		int cam_height_adj;  // 摄像头真实的安装高度

		int active_cnt_max;  // 每个点active_cnt的最大值
		int ref_support_th;  // 每个目标支撑点的128分比阈值
		int ref_people_th;   // 判定物体为人物的阈值（高度）
		int ref_rid_th;      // 物体的rid总数占总数的128分比高于该值，则认为该物体应以其对应的Floor物体为主
		int ref_rid_merge_th;// 当两个物体的图上区域上下差别小于该值，其左右有重叠，则合并两物体
		int ref_merge_th_uv; // 当两个物体所在的图像区域的InnerDif高于该值，则合并这两个物体
		int ref_merge_th_xy; // 当两个物体所在的物理区域的InnerDif高于该值，则合并这两个物体

		int pt_neighbor_th;  // 相邻两点物理坐标距离的阈值
		int pt_dep_dif_th;   // 该点与背景点的深度平均差异小于该值，则认为人员靠近背景足够近，能够张贴粘贴物
		
		int samples_per_pix; // 每个像素的样本个数
		int dif_thresh_grey; // 光学前景灰度与背景差异的阈值
		int dif_thresh_grad; // 光学前景梯度阈值
		int dif_thresh_shad; // 光学前景阴影判定阈值，该阈值用dif_thresh_grey的128分比表示，该值应大于128
		int dif_thresh_hue;  // 光学前景色度与背景差异的阈值
		int dif_thresh_dep;  // 深度前景深度与背景差异的阈值
		int bg_cnt_thresh;   // 与多少个样本点匹配就认为是背景
		int n_subsampling;   // 判定是背景后，更新进背景样本RGB的概率为1/n_subsampling
		int n_subsampling_d; // 判定是背景后，更新进背景样本深度的概率为1/n_subsampling_d
		int neighbor_span;   // 更新邻居的邻域大小

		int plus_step;       // 增益检测的步长
		int plus_cnt;        // 增益检测向上下探测的次数
		int try_step;        // DepthTryForeGround使用的探测步长

		int cam_h_low;       // 摄像头高度的下限
		int cam_h_high;      // 摄像头高度的上限
		int fu,fv;           // 图像坐标系与摄像头坐标系间转换的系数

		int track_uv_th;     // 目标跟踪时，当两个目标的图像矩形距离小于该值时认为是同一目标
		int track_xy_th;     // 目标跟踪时，当两个目标的物理矩形距离小于该值时认为是同一目标
		int track_height_th; // 目标分离时，要求分离后的目标高度与原高度之差不能高于该值，该值为128分比
		int track_inner_th;  // 目标分离时，要求分离后的目标inner距离必须大于该值
		int track_max_life;  // 目标跟踪时，life的最大值
		int track_still_th;  // 目标跟踪时，移动幅度小于该值认为物体没有运动
		int track_wander_th; // 目标离开原地多远，才认为他有进入WANDER状态的潜质
		int track_still_cnt_th; // 目标进入still状态多少帧，就可以将其作为WANDER的一站了
		int track_run_th;    // 目标跟踪时，速度大于该值认为物体在跑步，毫米/秒
		int track_near_th;   // 目标跟踪时，两人物距离低于该值则人物两目标距离太近
		int track_owner_th;  // 目标跟踪时，物体与人物间距离低于该值则人物两目标距离太近
		int track_owner_cnt_th; // 物体更换owner的次数低于该值，则将其still_cnt清零，避免过早的报遗留物------------------------
		int track_down_th;   // 目标高度降低后，其所占区域的宽/长不大于其站立时宽/长的比例，就认定该物体是下蹲，128分比

		int floor_pt_th;        // 当一个物体的深度前景点百分比低于该阈值时，认为是平面体
		int people_veri_th;     // 当一个物体的最高速度高于该值后，才认定它是人
		int people_area_th;     // 当一个目标的实际面积高于该值，才有可能认定为人
		int people_veri_cnt_th; // 当一个目标被检测出的次数高于该值，才有可能认定为人

		int people_run_th;      // 判定一个人物跑步的阈值（秒数），一旦报了跑步这么多秒内不能再报跑步
		int people_lie_th;      // 判定一个人物摔倒的阈值，高度128分比
		int people_lie_cnt_th;  // 判定一个人物摔倒的阈值，秒数
		int people_lie_long_th; // 判定一个人长时间倒地的阈值，秒数
		int people_stay_th;     // 判定一个物体/人物滞留的阈值，秒数
		int people_wander_dis_th;//判定一个人徘徊的阈值，总移动距离，毫米
		int people_follow_th;   // 判定两个人物相互跟踪的阈值，秒数
		int people_fight_th;    // 判定两个人物打架的阈值
		int people_fight_det_th;// 判断两人物打架：如果两次合并间的检测次数少于该值，则认为打架
		int people_down_th;     // 判定一个人下蹲的阈值，高度128分比
		int people_down_cnt_th; // 判定一个人下蹲的阈值，秒数
		int object_still_th;    // 判定一个物体遗留的阈值，秒数

		int obj_track_th;       // 判定新检测出的物体可以合并到已跟踪物体的阈值

		int frame_ignore;    // 在初始化前，首先忽略的帧数
		int frame_init;      // 在初始化时，构建地面等工作需要处理的帧数

		int light_up_th;     // 亮度过高的阈值
		int light_down_th;   // 亮度过低的阈值
		int light_thresh;    // 亮度变化的阈值
		int blur_thresh;     // 摄像头模糊的阈值（锐度过低）低于该值认为模糊
		int dep_valid_thresh;// 有效深度值个数阈值，128分比
		int cover_thresh;    // 判定摄像头遮挡的阈值，即平均深度值低于该值则认为遮挡
		int floor_cnt_thresh;// 判定地面点个数过低的阈值,地面点的千分比
		int cam_move_thresh;  // 判定摄像头移动的阈值,128分比
		int door_fg_thresh;  // 判定开门的阈值，前景点的128分比
		int cover_span;      // 判断摄像头遮挡或全黑时，需要累积的帧数

		int region_in_vio_th; // 进入导致区域违规时，前面人数多少帧没变，则认为是强行进入
		int region_pre_cnt_th;// 区域的pre_people_cnt维持多少帧，会将其变为cur_cnt_frames
		int region_fbuf_th;   // 区域内的人员数低于people_down多少帧后，报人数过低事件

		int ct_max_life;     // 记录的粘贴物最大生命值

		// region settings---------------------------------------------------
		Region valid_region;             // 有效区域
		Region regions[REGION_MAX_CNT];  // 编号区域
		
		// 参数别名----------------------------------------------------------
		int frame_bytes;     // 一个RGB帧需要的内存
		int frame_len;       // 一个RGB帧的点数
		int frame_line_bytes;// 一个RGB帧行的内存占用量

		// 格式转换相关的---------------------------------------------------
		unsigned short tab_r[256];
		unsigned short tab_g[256];
		unsigned short tab_b[256];

	} Depth;
	
// 外部接口===============================================================================================
//初始化Depth数据结构
// pDep: 深度模块所需要的数据结构
// width，height：图像的宽度和高度
// path:Depth模块配置文件路径
// 成功时返回true，否则返回false，返回false时需要检查配置文件的路径，以及配置的参数是否合理
bool DepthInit( Depth* pDep, int width, int height, char* path );

// 释放Depth数据结构的资源
// pDep: Depth对象的指针
void DepthUninit( Depth* pDep );

// 对深度/色彩图像进行处理
// pDep: 深度模块所需要的数据结构
// pRGB: RGB格式输入图像
// pDepth:深度格式输入图像
int DepthProcess( Depth * pDep, unsigned char * pRGB, unsigned short * pDepth );

int DepthChangeStatus( Depth * pDep, int status );

// 内部使用===============================================================================================

// 删除深度图中的异常点-------------------------------------------------------------------------
void FilterOutDepth( Depth * pDep, unsigned short * pDepth );

// 检测图像质量、遮挡等-------------------------------------------------------------------------
void CheckCamQuality( Depth * pDep );
int  AcutanceAndGrey( Depth * pqua, unsigned char * pFrameGrey ); // 计算锐度，返回平均灰度

// 寻找地面和标定摄像机参数---------------------------------------------------------------------
void  SearchFloor( Depth* pDep );
float DecideCamParamMin2( Depth * pDep, float * VD );
void  RefineFloor( Depth * pDep );
void  ApplyCamParam( Depth * pDep, float * VD );
void  CalcAttr( Depth* pDep );

void SearchPlane( Depth * pDep );
bool CalcDirection( float * CP1, float * CP2, float * CP3, float * CP4, float * VD, float thresh );
void CalcPT( float pt[3], unsigned short dep, int u, int v, int fu, int fv );

// 深度处理流程---------------------------------------------------------------------------------
bool DetectTargets( Depth * pDep, short * map );
int  DetectPolarTargets( Depth * pDepth, int region_idx );
void RefinePolarTargets( Depth * pDepth );
void ReverseMap( Depth * pDep );
bool RefineTargets( Depth * pDep, short * map );
void TrackTargets( Depth * pDep );

// 结合光学前景检测-----------------------------------------------------------------------------
void DepthDetectMotion(Depth* pDep);
void DepthBkgInit(Depth* pDep,int idx);
int  DepthTryForeGround( Depth* pDep, int plus );
int  DepthDetForeGround( Depth* pDep, int plus );
void DepthBkgUpdate(Depth* pDep);
void DepthRefineFG( Depth* pDep );

// 已经报出的平面物体跟踪-----------------------------------------------------------------------
int  CheckTrack( Depth * pDep, int idx );
void HandleTrack( Depth * pDep );
int  Different( Depth * pDep, unsigned char * l, unsigned char * r, Rect rrect );
void GenBG( Depth * pDep );

// 工具函数-------------------------------------------------------------------------------------
void DepthRGB2Grey( Depth * pDep, unsigned char* pDFrameGrey, unsigned char * pSFrame );
void DepthGenGrad( Depth * pDep );
int  DepthRectDif( Rect l, Rect r, Rect * pcommon );  // 计算两个区域的重合度，0~100
int  DepthInnerDif( Rect l, Rect r );  // 计算两个区域的重合度，0~100，这次以二者中小的为分母
int  DepthOuterDif( Rect l, Rect r );   // 计算两个区域的重合度，0~100，这次以二者中小的为分母
int  RectDistance( Rect l, Rect r );    // 计算两个区域间的距离，以四个点之间最近的那对为准
int  UpdateRegion( Depth* pDep );       // 计算设置好的区域的x、y坐标，如果成功返回1，否则返回-1
Rect MergeRect( Rect l, Rect r );      // 合并两个rect
Rect AndRect( Rect l, Rect r );        // 取两个rect的与运算
int  CalcSpeed( Rect pre, Rect now, short * spx, short * spy );    // 计算物体的运动速度，已知物体所占区域
void HandleRegionPeopleCnt( Depth * pDep, Region * pregion, int people_cnt );    // 处理某个区域的人数相关的东西
void CalIntegralFloor( long long* pInte, int * pFloorFGInte, int * pFloorPrjInte, FloorAttr* pFloor, int width, int height ); // 计算高程图的积分图
void World2UV( Depth * pDep, int * u, int * v, int x, int y, int h );        // 世界坐标系转换为图上坐标系
bool IsCross(Point pt1, Point pt2, Point line_s, Point line_e, Point dir);
int  FloorDif( Depth * pDep, Rect curr, Rect last );
int  IsInRect( int u, int v, Rect rct );

// 调试用---------------------------------------------------------------------------------------
void SaveID( char * path, FloorAttr * pAttr, short * map, int width, int height );
void SaveIDImg( char * path, ImgAttr * pAttr, short * map, int width, int height );

#ifdef __cplusplus
}
#endif

#endif
