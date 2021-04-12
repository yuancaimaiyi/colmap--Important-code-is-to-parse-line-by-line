# colmap--Important-code-is-to-parse-line-by-line
教你一点点掌握视觉三维重建-colmap 重要代码逐行解析(本人利用下班和周末时间update,so 速度会慢)
#教你一点点掌握视觉三维重建-colmap 重要代码逐行解析
*这里以colmap 框架为准,主要对其重要环节代码结合自己的想法进行逐一讲解,colmap 作为目前state-of-the-art 的视觉重建pipeline,本人将其代码分为两个大环节:前端和后端.前端主要是特征提取和匹配,后端包括三角化,Register,BA等环节.*

## 目录

1. 特征提取

2. 特征匹配

3. 三角化或前方交会

4. 运动恢复

5. 光束法平差
