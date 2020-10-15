# 通过模板技术视觉抓取

- 捕获Kinect2相机的彩色图和深度图 PhotoCapture
    - 实时预览彩色图
    - 实时预览深度图
    - 实时预览点云图

- 通过彩色图和深度图制作模板点云
- 需要相机内参 计算点云xy的值
- 01-PhotoCapture.cpp 实时显示深度图及彩色图  
- 01-PhotoCapture2.cpp 实时显示深度图及彩色图及点云图 空格保存数据
- 02-TemplateCloudFilter.cpp 把场景通过直通滤波 过滤出需要的点云图 把盒子模板点云单独过滤出来
- 03-TemplateRtMaker.cpp 眼在手外获取模板的位姿矩阵 确定机械臂从上还是横着抓 template_Rt.xml
    通过用过在点云图上用鼠标选取的点生成夹抓抓的取物体的中心点及夹抓的位姿(夹抓竖着抓还是横这抓)
  -控制台启动命令需要传点云文件  ./03-TemplateRtMaker ./output/table_scene_0_ds.pcd
  - shift + 鼠标左键可获取点云坐标,选择要抓取物体的3或4个点 按w保存 物品旋转矩阵与平移矩阵
  - space空格清除所有点及形状,z 清除上一个点 ,a通过点绘制坐标系,w保存
  - 根据保存的旋转矩阵与平移矩阵就可以单独抓取原位的物体
- 04-TestGrabTemplate.cpp  尝试根据已有的数据抓取目标; 抓取 03-TemplateRtMaker.cpp 生成的物品 template_Rt.xml

- 05-TemplateAlignment.cpp 眼在手外磨版匹配,生成场景中对应目标的位姿 启动 ./data1/object_templates.txt ./data1/person.pcd
  通过模板点云 ./data1/object_templates.txt 匹配场景点云 ./data1/person.pcd
  得到场景点云中物品的位置  "./template2target_Rt.xml";
- 06-GrabTarget.cpp  眼在手外抓取物品  点云模板抓取
     // 模板位姿 机械臂要抓取的位姿  竖抓还是横抓
    const string templateRtPath = "./output/template_Rt.xml";
    //外参文件 确定相机坐标 机械臂坐标
    const string ex_calib_file_path = "./assets/calibration_ex_params.yml";
    // 目标盒子所在的位置及姿态
    const string template2target_Rt_filepath = "./template2target_Rt.xml";
    通过物品的位置 "./template2target_Rt.xml"; 机械臂抓取物品
    原理:
    物品在任何位置抓取,自动匹配模板 1. 首先执行把物品放置深度相机下执行 05-TemplateAlignment.cpp 得到变化矩阵,而后执行06-GrabTarget.cpp进行抓取
    抓取的物品的位姿改变需要重新执行 05-TemplateAlignment.cpp 得到变化矩阵在进行抓取,或把05-TemplateAlignment.cpp 集成进这个06-GrabTarget.cpp
    类中实现品在任何位置的位姿变化的抓取[实时获取物体位姿,实时获取变化矩阵进行抓取操作]


     传入的参数  ./output/table_scene_0_ds.pcd
     控制台命令  ./03-TemplateRtMaker ./output/table_scene_0_ds.pcd
     控制台降采样命令 pcl_voxel_grid table_scene_0_ds.pcd table_scene_0_ds.pcd -leaf   0.003,0.003,0.003
     控制台显示点云图 pcl_viewer table_scene_0_ds.pcd
     控制台显示点云图 pcl_viewer -use_point_picking table_scene_0_ds.pcd    [shift + 鼠标左键可获取点云坐标]
     查看历史操作的命令 history
     /bin pcl_viewer ./data/chef.pcd
     /bin pcl_viewer ./data/rs1.pcd
     /bin pcl_viewer ./data/chef.pcd ./data/rs1.pcd
     /data1  pcl_viewer person.pcd object_template_*pcd  // 加载多个点云文件
     按 u 标注尺
     按 1 切换颜色
     按 2 x方向颜色
     按 3 y方向颜色
     按 4 z方向颜色


​     
​     
