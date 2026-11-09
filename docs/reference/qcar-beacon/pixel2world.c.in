/* pixel2world.c：相机坐标转换。创建日期：2025年5月29日；作者：gongheng。 */

#include "pixel2world.h"
// 相机标定参数：应按实际相机和安装位置重新标定。


// 相机内参矩阵 K，使用当前代码中的存储顺序。
float IM[3][3] =
{
    {89.105116922854010f, 0.0f, 90.612120843053180f},
    {0.0f, 1.647561248229724e+02f, 60.284871440836800f},
    {0.0f, 0.0f, 1.0f}
};

// 相机标定旋转向量，单位为弧度。
float rvec[3] = {-1.344993599976673f, 0.009972960246542f, -0.008196928741952f};

// 平移向量 tvec，单位为米。
float tvec[3] = {-1.799756252199414e+02f, 8.860384219128870f, 7.516845303458088e+02f};

// 畸变系数，顺序为 k1、k2、p1、p2、k3。
float distCoeffs[5] = {-0.445098928704561f, 0.161184521577233f, 0.0f, 0.0f, 0.0f};

// 初始偏移量，单位为毫米。
float OriginOffset[2] = {-180.00f, 660.00f};

//float IM[3][3] =
//{
//    {81.848041841567620f, 0.0f, 93.354509750262650f},
//    {0.0f, 1.100699497260044e+02f, 64.588671625725700f},
//    {0.0f, 0.0f, 1.0f}
//};
//
// 历史相机标定旋转向量，单位为弧度。
//float rvec[3] = {-1.352909455923476f, -0.003993243872685f,0.009885473761763f};
//
// 历史标定的平移向量，单位为米。
//float tvec[3] = {-2.735431673890271e+02f, 81.624536620235130f, 6.821222892726122e+02f};
//
// 历史标定的畸变系数。
//float distCoeffs[5] = {-0.445098928704561f, 0.161184521577233f, 0.0f, 0.0f, 0.0f};
//
// 历史标定的初始偏移量，单位为毫米。
//float OriginOffset[2] = {-270.00f, 580.00f};

// 像素坐标转换算法。

void skewSymmetric(float n[3], float Kmat[3][3])
{
    Kmat[0][0] = 0.0f;     Kmat[0][1] = -n[2]; Kmat[0][2] = n[1];
    Kmat[1][0] = n[2];     Kmat[1][1] = 0.0f;  Kmat[1][2] = -n[0];
    Kmat[2][0] = -n[1];    Kmat[2][1] = n[0];  Kmat[2][2] = 0.0f;
}

void rodrigues(float rvec[3], float R[3][3])
{
    float theta = sqrtf(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    if (theta < 1e-6f) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R[i][j] = (i == j) ? 1.0f : 0.0f;
        return;
    }

    float n[3] = {rvec[0]/theta, rvec[1]/theta, rvec[2]/theta};
    float Kmat[3][3], K2[3][3];
    skewSymmetric(n, Kmat);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            K2[i][j] = Kmat[i][0]*Kmat[0][j] + Kmat[i][1]*Kmat[1][j] + Kmat[i][2]*Kmat[2][j];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = ((i == j) ? 1.0f : 0.0f) + sinf(theta)*Kmat[i][j] + (1.0f - cosf(theta))*K2[i][j];
}

void undistortPoint(float u, float v, float undistorted[2])
{
    float fx = IM[0][0], fy = IM[1][1], cx = IM[0][2], cy = IM[1][2];
    float x = (u - cx) / fx;
    float y = (v - cy) / fy;

    float x_undist = x, y_undist = y;
    for (int iter = 0; iter < 5; ++iter) {
        float r2 = x_undist*x_undist + y_undist*y_undist;
        float radial = 1.0f + distCoeffs[0]*r2 + distCoeffs[1]*r2*r2 + distCoeffs[4]*r2*r2*r2;
        float dx = 2.0f*distCoeffs[2]*x_undist*y_undist + distCoeffs[3]*(r2 + 2.0f*x_undist*x_undist);
        float dy = distCoeffs[2]*(r2 + 2.0f*y_undist*y_undist) + 2.0f*distCoeffs[3]*x_undist*y_undist;

        x_undist = (x - dx) / radial;
        y_undist = (y - dy) / radial;
    }

    undistorted[0] = x_undist;
    undistorted[1] = y_undist;
}

void pixel2world(float u, float v, float world[3],ContourInfo* contourinfo)
{
    if(u < 0 || u > Umax || v < 0 || v > Vmax)
    {
        world[0] = world[1] = world[2] = 0.0f;
        return;
    }

    float norm[3];
    float undistorted[2];
    undistortPoint((float)u, (float)v, undistorted);

    norm[0] = undistorted[0];
    norm[1] = undistorted[1];
    norm[2] = 1.0f;

    float R[3][3];
    rodrigues(rvec, R);

    float A[2][2] = {
        {norm[0]*R[2][0] - R[0][0], norm[0]*R[2][1] - R[0][1]},
        {norm[1]*R[2][0] - R[1][0], norm[1]*R[2][1] - R[1][1]}
    };

    float B[2] = {
        tvec[0] - norm[0]*tvec[2],
        tvec[1] - norm[1]*tvec[2]
    };

    float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabsf(det) < 1e-8f) {
        world[0] = world[1] = world[2] = 0.0f;
        return;
    }

    world[0] = ((B[0]*A[1][1] - B[1]*A[0][1]) / det + OriginOffset[0])*0.001;
    world[1] = (-(A[0][0]*B[1] - A[1][0]*B[0]) / det + OriginOffset[1])*0.001;

    //world[0] = -(A[0][0]*B[1] - A[1][0]*B[0]) / det + OriginOffset[0];
    //world[1] = (B[0]*A[1][1] - B[1]*A[0][1]) / det + OriginOffset[1];
    if(v<25||world[1]<0)
    {
        contourinfo->distance=100;
        contourinfo->angle=0;
    }
    else
    {
        contourinfo->distance=sqrtf(world[0] * world[0] + world[1] * world[1]);
        contourinfo->angle=atan2f(world[1],world[0])*RAD2DEG;
        contourinfo->x=world[0];
        contourinfo->y=world[1];
    }
    world[2] = 0.0f;
//    printf("x:%.2f,y:%.2f,d:%.2f,a:%.2f\r\n",world[0],world[1],contourinfo->distance,contourinfo->angle);
}

// 像素坐标到世界坐标的转换。




