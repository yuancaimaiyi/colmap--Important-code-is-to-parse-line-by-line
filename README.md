# colmap--Important-code-is-to-parse-line-by-line
教你一点点掌握视觉三维重建-colmap 重要代码逐行解析(本人利用下班和周末时间update,so 速度会慢)

## 个人改进效果如下: <br />
https://www.bilibili.com/video/BV1Dv411774G   FSBA  <br />
https://www.bilibili.com/video/BV1Sp4y1b76x    facing-camera motion fusion gps <br />
https://www.bilibili.com/video/BV1UK4y127Nb    backward-camera motion  fusion gps <br />  
### 2022.3.3  
**技术路线图**  
![Image text]()

### 2021.5.28
**从计算机视觉(slam)和摄影测量两个维度进行BA算法原理推导**
![contents](https://github.com/yuancaimaiyi/colmap--Important-code-is-to-parse-line-by-line/blob/main/%E4%BB%8E%E8%AE%A1%E7%AE%97%E6%9C%BA%E8%A7%86%E8%A7%89(slam)%E5%92%8C%E6%91%84%E5%BD%B1%E6%B5%8B%E9%87%8F%E4%B8%A4%E4%B8%AA%E7%BB%B4%E5%BA%A6%E8%BF%9B%E8%A1%8Cba%20%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E6%8E%A8%E5%AF%BC.pdf)   
### 2021.



# 教你一点点掌握视觉三维重建-colmap 重要代码逐行解析

*这里以colmap 框架为准,主要对其重要环节代码结合自己的想法进行逐一讲解,colmap 作为目前state-of-the-art 的视觉重建pipeline,本人将其代码分为两个大环节:前端和后端.前端主要是特征提取和匹配,后端包括三角化,Register,BA等环节.*

## 目录

1. 特征提取

2. 特征匹配

3. 三角化或前方交会

4. 运动恢复

5. 光束法平差及其改进

## 前端

### feature extracting 

- sift-GPU 算法

        待写

- DSP-SIFT算法

         待写

  Domain-size pooling SIFT 是从多个尺度的SIFT 描述子进行一个平均,参考论文:***Domain-Size Pooling in Local Descriptors and Network Architectures***,其已被证明优于其他SIFT算法的变种及一些深度学习算子.

### feature matching 

- 匹配方法

         待写

- 几何验证-剔除outliers

       1.  对于标定相机，，利用E/F 、H/F 、H/E的内点个数比值来决定使用哪种模型来剔除误点

           代码最终 inlier_matches = ExtractInlierMatches(matches,模型内点数，模型mask)

```C++
  //  对于标定相机 利用 E/F H/F H/E 的内点比值来决定使用那种模型来剔除outliers

  const double E_F_inlier_ratio =
      static_cast<double>(E_report.support.num_inliers) /
      F_report.support.num_inliers;
  const double H_F_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      F_report.support.num_inliers;
  const double H_E_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      E_report.support.num_inliers;
```

       2 . 对于非标相机，利用F 矩阵模型 

```C++
  // 非标相机估计 H/F 的比值即可,E 矩阵无法得到
  const double H_F_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      F_report.support.num_inliers;

  if (H_F_inlier_ratio > options.max_H_inlier_ratio) {
    config = ConfigurationType::PLANAR_OR_PANORAMIC;
  } else {
    config = ConfigurationType::UNCALIBRATED;
  }

  inlier_matches = ExtractInlierMatches(matches, F_report.support.num_inliers,
                                        F_report.inlier_mask);
```

**对于外点的剔除,除了利用ransac 估计模型来剔除outliers ,还有可改进的思路吗?众所周知,ransac 是每次进行随机抽样,重新计算模型,这样就比较耗时.**
## 后端

### trianglulation 

colmap 代码中，三角化分为以下几个函数：

1.  两帧三角化 ，即使用SVD 分解求解P（M*P=m),代码如下:

```C++
// proj_matrix1 ,proj_matrix2 分别是世界系到图像系的投影矩阵
Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& proj_matrix1,
                                 const Eigen::Matrix3x4d& proj_matrix2,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2) {
  Eigen::Matrix4d A;

  A.row(0) = point1(0) * proj_matrix1.row(2) - proj_matrix1.row(0);
  A.row(1) = point1(1) * proj_matrix1.row(2) - proj_matrix1.row(1);
  A.row(2) = point2(0) * proj_matrix2.row(2) - proj_matrix2.row(0);
  A.row(3) = point2(1) * proj_matrix2.row(2) - proj_matrix2.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);

  return svd.matrixV().col(3).hnormalized();
}
```

  2 . 多帧三角化 ，即是求解超定方程组.从多个观测值恢复3D 场景结构，三角化需要满足以下两个条件：

        1)  所有观测值需要满足 cheirality constraint (什么是cheirality constraint ，即是正景深约束，多视图几何书中分解E 矩阵的时候，会得到四种结果-R,t，但是重构点在相机前方的结果只有一个）

       2)  观测对之间有足够的三角化角度

      对于求解的结果需要满足以上两个条件,代码如下

      i .条件1 : xyz  cheirality constraint 

```C++
    for (const auto& pose : pose_data) {
      if (!HasPointPositiveDepth(pose.proj_matrix, xyz)) {
        return std::vector<M_t>();
      }
    }
```

条件2: 像对之间需足够的三角化角度(阈值为min_tri_angle)

```C++
    for (size_t i = 0; i < pose_data.size(); ++i) {
      for (size_t j = 0; j < i; ++j) {
        const double tri_angle = CalculateTriangulationAngle(
            pose_data[i].proj_center, pose_data[j].proj_center, xyz);
        if (tri_angle >= min_tri_angle_) {
          return std::vector<M_t>{xyz};
        }
      }
    }
```

3. 鲁棒性LO-RANSAC 多帧三角化,其主要是剔除一些错误的匹配点,这个也是colmap 框架三角化一个特色,代码如下:

```C++
bool EstimateTriangulation(
    const EstimateTriangulationOptions& options,
    const std::vector<TriangulationEstimator::PointData>& point_data,
    const std::vector<TriangulationEstimator::PoseData>& pose_data,
    std::vector<char>* inlier_mask, Eigen::Vector3d* xyz) {
  CHECK_NOTNULL(inlier_mask);
  CHECK_NOTNULL(xyz);
  CHECK_GE(point_data.size(), 2);
  CHECK_EQ(point_data.size(), pose_data.size());
  options.Check();

  // Robustly estimate track using LORANSAC.
  LORANSAC<TriangulationEstimator, TriangulationEstimator,
           InlierSupportMeasurer, CombinationSampler>
      ransac(options.ransac_options);
  ransac.estimator.SetMinTriAngle(options.min_tri_angle);
  ransac.estimator.SetResidualType(options.residual_type);
  ransac.local_estimator.SetMinTriAngle(options.min_tri_angle);
  ransac.local_estimator.SetResidualType(options.residual_type);
  const auto report = ransac.Estimate(point_data, pose_data);
  if (!report.success) {
    return false;
  }

  *inlier_mask = report.inlier_mask;
  *xyz = report.model;

  return report.success;
}
```

### Register or motion recover

- 基本矩阵FundamentalMatrix 

     基本矩阵的求解从以下公式出发:
        $$
\mathbf{x}^{\prime T} F \mathbf{x}=0
$$

以上公式可以写成包含9个未知数的线性齐次方程,如下:

          $\begin{aligned}
&\mathbf{u}_{i}^{T} \mathbf{f}=0\\
&\begin{aligned}
\mathbf{u}_{i} &=\left[u_{i} u_{i}^{\prime}, v_{i} u_{i}^{\prime}, u_{i}^{\prime}, u_{i} v_{i}^{\prime}, v_{i} v_{i}^{\prime}, v_{i}^{\prime}, u_{i}, v_{i}, 1\right]^{T} \\
\mathbf{f} &=\left[F_{11}, F_{12}, F_{13}, F_{21}, F_{22}, F_{23}, F_{31}, F_{32}, F_{33}\right]^{T}
\end{aligned}
\end{aligned}$
  读过MVG的人都知道基本矩阵由于一个尺度不确定和其秩等于2,故其DOF 等于7.所以估计基本矩阵的算法根据不忽略rank-2 constraint 和忽略rank-2 constraint,求解方法可分为7点法和8点法或者more points 法

  1)  不忽略rank-2 constraint  的7点法,其核心是确定lambda 和mu,其中lambda + mu =1;由上式可以看出,9个未知数,7个方程,明显是无法求解,那么由于rank=2 的限制,那么F 矩阵的null space 可以由f1和f2的线性组合表示,故有下式(参考论文-***Determining the Epipolar Geometry and its Uncertainty: A Review, International Journal of Computer Vision***):

  $\mathbf{F}=\alpha \mathbf{F}_{1}+(1-\alpha) \mathbf{F}_{2}$

  $\operatorname{det}\left[\alpha \mathbf{F}_{1}+(1-\alpha) \mathbf{F}_{2}\right]=0$

  代码如下:

```C++
std::vector<FundamentalMatrixSevenPointEstimator::M_t>
FundamentalMatrixSevenPointEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), 7);
  CHECK_EQ(points2.size(), 7);

 
  Eigen::Matrix<double, 7, 9> A;
  for (size_t i = 0; i < 7; ++i) {
    const double x0 = points1[i](0);
    const double y0 = points1[i](1);
    const double x1 = points2[i](0);
    const double y1 = points2[i](1);
    A(i, 0) = x1 * x0;
    A(i, 1) = x1 * y0;
    A(i, 2) = x1;
    A(i, 3) = y1 * x0;
    A(i, 4) = y1 * y0;
    A(i, 5) = y1;
    A(i, 6) = x0;
    A(i, 7) = y0;
    A(i, 8) = 1;
  }

  // 9 个未知数,7个方程, 所以我们有两个null space .
  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 9>> svd(A, Eigen::ComputeFullV);
  const Eigen::Matrix<double, 9, 9> f = svd.matrixV();
  Eigen::Matrix<double, 1, 9> f1 = f.col(7);
  Eigen::Matrix<double, 1, 9> f2 = f.col(8);

  f1 -= f2;
 // 很明显,方程个数不够,无法求解,所以我们必须增加约束
 // det(F) = det(lambda * f1 + (1 - lambda) * f2)
 // 其中lambda + mu = 1

  const double t0 = f1(4) * f1(8) - f1(5) * f1(7);
  const double t1 = f1(3) * f1(8) - f1(5) * f1(6);
  const double t2 = f1(3) * f1(7) - f1(4) * f1(6);
  const double t3 = f2(4) * f2(8) - f2(5) * f2(7);
  const double t4 = f2(3) * f2(8) - f2(5) * f2(6);
  const double t5 = f2(3) * f2(7) - f2(4) * f2(6);

  Eigen::Vector4d coeffs;
  coeffs(0) = f1(0) * t0 - f1(1) * t1 + f1(2) * t2;
  coeffs(1) = f2(0) * t0 - f2(1) * t1 + f2(2) * t2 -
              f2(3) * (f1(1) * f1(8) - f1(2) * f1(7)) +
              f2(4) * (f1(0) * f1(8) - f1(2) * f1(6)) -
              f2(5) * (f1(0) * f1(7) - f1(1) * f1(6)) +
              f2(6) * (f1(1) * f1(5) - f1(2) * f1(4)) -
              f2(7) * (f1(0) * f1(5) - f1(2) * f1(3)) +
              f2(8) * (f1(0) * f1(4) - f1(1) * f1(3));
  coeffs(2) = f1(0) * t3 - f1(1) * t4 + f1(2) * t5 -
              f1(3) * (f2(1) * f2(8) - f2(2) * f2(7)) +
              f1(4) * (f2(0) * f2(8) - f2(2) * f2(6)) -
              f1(5) * (f2(0) * f2(7) - f2(1) * f2(6)) +
              f1(6) * (f2(1) * f2(5) - f2(2) * f2(4)) -
              f1(7) * (f2(0) * f2(5) - f2(2) * f2(3)) +
              f1(8) * (f2(0) * f2(4) - f2(1) * f2(3));
  coeffs(3) = f2(0) * t3 - f2(1) * t4 + f2(2) * t5;

  Eigen::VectorXd roots_real;
  Eigen::VectorXd roots_imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs, &roots_real, &roots_imag)) {
    return {};
  }

  std::vector<M_t> models;
  models.reserve(roots_real.size());

  for (Eigen::VectorXd::Index i = 0; i < roots_real.size(); ++i) {
    const double kMaxRootImag = 1e-10;
    if (std::abs(roots_imag(i)) > kMaxRootImag) {
      continue;
    }

    const double lambda = roots_real(i);
    const double mu = 1;

    Eigen::MatrixXd F = lambda * f1 + mu * f2;

    F.resize(3, 3);

    const double kEps = 1e-10;
    if (std::abs(F(2, 2)) < kEps) {
      continue;
    }

    F /= F(2, 2);

    models.push_back(F.transpose());
  }

  return models;
}

```

     2) 忽略rank-2 constraint   就成了8点法或者更多点法,这个时候我们就可以使用最小二乘来求解,即是

  $\min _{\mathbf{F}} \sum_{i}\left(\tilde{\mathbf{m}}_{i}^{\prime T} \mathbf{F} \tilde{\mathbf{m}}_{i}\right)^{2}$

  $\min _{\mathrm{f}}\left\|\mathbf{U}_{n} \mathbf{f}\right\|^{2}$

  现在f只 受一个未知尺度的影响,为了避免f=0,我们必须增加约束,即使SVD 中V 的最后一个特征值等于0(详见<<计算机视觉中的多视图几何>>书本281页),代码如下:

```C++
std::vector<FundamentalMatrixEightPointEstimator::M_t>
FundamentalMatrixEightPointEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  // 中心归一化是为了数值稳定性
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  //解决线性方程 x2' * F * x1 = 0.
  Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
  for (size_t i = 0; i < points1.size(); ++i) {
    cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
    cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
    cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
  }

  // SVD 分解
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
      cmatrix, Eigen::ComputeFullV);
  const Eigen::VectorXd cmatrix_nullspace = cmatrix_svd.matrixV().col(8);
  const Eigen::Map<const Eigen::Matrix3d> ematrix_t(cmatrix_nullspace.data());

  // 增加约束,即SVD 中的V 的特征向量最后一个等于0 
  Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(
      ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = fmatrix_svd.singularValues();
  singular_values(2) = 0.0;
  const Eigen::Matrix3d F = fmatrix_svd.matrixU() *
                            singular_values.asDiagonal() *
                            fmatrix_svd.matrixV().transpose();

  const std::vector<M_t> models = {points2_norm_matrix.transpose() * F *
                                   points1_norm_matrix};
  return models;
}
```

- 本质矩阵

     1) 5点法求解

        待写

     2) 8点法求解

            待写

- 单应矩阵

      单应矩阵的求解相对于本质和基本就相对简单,在colmap 代码中直接利用DLT算法直接求解,理论公式如下:

      $\left(\begin{array}{l}u \\ v \\ 1\end{array}\right)=H\left(\begin{array}{l}x \\ y \\ 1\end{array}\right)$

其中H:

$H=\left[\begin{array}{lll}h_{1} & h_{2} & h_{3} \\ h_{4} & h_{5} & h_{6} \\ h_{7} & h_{8} & h_{9}\end{array}\right]$

线性化展开得到:

$-h_{1} x-h_{2} y-h_{3}+\left(h_{7} x+h_{8} y+h_{9}\right) u=0
$

$-h_{4} x-h_{5} y-h_{6}+\left(h_{7} x+h_{8} y+h_{9}\right) v=0$

整理可得:

$A_{i}=\left(\begin{array}{ccccccccc}-x & -y & -1 & 0 & 0 & 0 & u x & u y & u \\ 0 & 0 & 0 & -x & -y & -1 & v x & v y & v\end{array}\right)
$

$h=\left(\begin{array}{cccccc}h_{1} & h_{2}  & h_{3} & h_{4} & h_{5} & h_{6} & h_{7} & h_{8} & h_{9}\end{array}\right)_{}$

利用最小二乘求解超定方程组便可求,代码如下:

```C++
std::vector<HomographyMatrixEstimator::M_t> HomographyMatrixEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  const size_t N = points1.size();

  // 中心归一化,数值稳定性
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  //构建方程
  Eigen::Matrix<double, Eigen::Dynamic, 9> A = Eigen::MatrixXd::Zero(2 * N, 9);

  for (size_t i = 0, j = N; i < points1.size(); ++i, ++j) {
    const double s_0 = normed_points1[i](0);
    const double s_1 = normed_points1[i](1);
    const double d_0 = normed_points2[i](0);
    const double d_1 = normed_points2[i](1);

    A(i, 0) = -s_0;
    A(i, 1) = -s_1;
    A(i, 2) = -1;
    A(i, 6) = s_0 * d_0;
    A(i, 7) = s_1 * d_0;
    A(i, 8) = d_0;

    A(j, 3) = -s_0;
    A(j, 4) = -s_1;
    A(j, 5) = -1;
    A(j, 6) = s_0 * d_1;
    A(j, 7) = s_1 * d_1;
    A(j, 8) = d_1;
  }

  // SVD 求解
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
      A, Eigen::ComputeFullV);

  const Eigen::VectorXd nullspace = svd.matrixV().col(8);
  Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data()); // 单应矩阵

  const std::vector<M_t> models = {points2_norm_matrix.inverse() *
                                   H_t.transpose() * points1_norm_matrix};
  return models;
}
```

- Estimate  Relative Pose (2D-2D)

       待写

- Estimate  Absolute Pose(2D-3D)

       待写

### Bundle adjustment

- 经典的重投影误差

           经典的bundlle adjsutment 采用最小二乘法估计相机位姿和3D点(更多的bundle adjusment 历史发展参考本人之前的文章—-）其Key idea 如下：

            （1）开始估计一个初始值

            （2） 将估计的3D点坐标投影到像平面

            （3）比较像平面测量值和投影值

            （4） 最小化每张图像的（3）误差

BA 后结果的质量定量评定

 $\widehat{\Sigma}_{\widehat{x} \hat{x}}=\widehat{\sigma}_{0}^{2}\left(A^{\top} \Sigma_{l l}^{-1} A\right)^{-1}$

A ：雅可比矩阵  ；$\Sigma_{l l}^{-1}$ ：信息矩阵 

- pose 为常量的重投影误差

           待写

- 部分参数优化(如R or T)的重投影误差

          待写

- GPS 约束的重投影误差(SfM 融合gps)

           待写

- GCP 约束的重投影误差( Marker SfM)

         待写

- IMU 角度约束的重投影误差

          待写
