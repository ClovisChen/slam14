# 特征提取与匹配
检测FAST角点位置
根据角点位置计算BRIEF描述子
对描述子进行匹配，使用汉明距离
匹配点对筛选
# 求解相机运动
## 2d2d 对极约束
vector<Point2f>
E, F, H 从E中恢复出平移旋转信息，验证E = t ^R*scale
验证对极约束
## 三角测量
for ( DMatch m:matches )
{
  pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
  pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
}
//! a[2]  <-->  *[a+2]

## 3d2d PnP(BA需要K(内参矩阵))
P3P
BA
PnP： EPnP
VetexSE3Expmap 李代数位姿
VertexSBAPointXYZ 空间点位姿
EdgeProjectXYZ2UV 边投影方程

## 3d3d ICP(BA不用K， 因为没有2d)
* SVD
计算两组点的质心
根据优化问题计算旋转矩阵Ｒ
根据Ｒ，　计算ｔ
* 非线性优化方法
重写EdgeProjectXYZ2UV 边投影方程

.pt
//coordinates of the keypoints
