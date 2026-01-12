# 已知:

像素坐标: (u, v)(x:图像中的水平位置，范围 [0, width-1],v:图像中的垂直位置，范围 [0, height-1])
深度值: depth_value (传感器原始值, uint16,该像素点距离相机的实际距离（米）)
相机内参: fx, fy, cx, cy(fx:X方向焦距（像素）,fy:Y方向焦距（像素）,cx:主点X坐标（图像中心点X）,cy:主点Y坐标（图像中心点Y）,width:图像宽度,height:图像高度)，焦距 (fx, fy): 控制透视投影的缩放比例，主点 (cx, cy): 光轴与图像平面的交点，通常接近图像中心，用于将2D像素坐标反投影到3D空间
深度尺度: depth_scale (深度值转米的系数)

# 计算步骤:

rs.rs2_deproject_pixel_to_point()函数封装了下面的变化并考虑了畸变
Z_cam = depth_value * depth_scale        # 深度(米)
X_cam = (u - cx) * Z_cam / fx           # 右方向(米)
Y_cam = (v - cy) * Z_cam / fy           # 下方向(米)

# 结果: 相机坐标系下的3D点 [X_cam, Y_cam, Z_cam]

realsense d435i光学坐标系详情：
z:向前（深度方向）（垂直于镜头表面，从镜头中心指向被拍摄物体）
x:向右(图像坐标系的 u 轴（列）)
y:向下(图像坐标系的 v 轴（行）)
