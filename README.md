# colmap--Important-code-is-to-parse-line-by-line
教你一点点掌握视觉三维重建-colmap 重要代码逐行解析(本人利用下班和周末时间update,so 速度会慢)



# 教你一点点掌握视觉三维重建-colmap 重要代码逐行解析

*这里以colmap 框架为准,主要对其重要环节代码结合自己的想法进行逐一讲解,colmap 作为目前state-of-the-art 的视觉重建pipeline,本人将其代码分为两个大环节:前端和后端.前端主要是特征提取和匹配,后端包括三角化,Register,BA等环节.*

## 目录

1. 特征提取

2. 特征匹配

3. 三角化或前方交会

4. 运动恢复

5. 光束法平差

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
