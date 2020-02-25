#include "robotarm.h"
#include "DataControl/datacontrol.h"

Body::Body(){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;
}

Body::~Body(){}

void Body::ang2mat(double ang_z1, double ang_x, double ang_z2, double *mat, bool deg_flag)
{
    double z1, x, z2;
    if (deg_flag){
        z1 = ang_z1*M_PI/180.0;
        x = ang_x*M_PI/180.0;
        z2 = ang_z2*M_PI/180.0;
    }
    else{
        z1 = ang_z1;
        x = ang_x;
        z2 = ang_z2;
    }

    double Rz1[9] = {cos(z1), -sin(z1), 0, sin(z1), cos(z1), 0, 0, 0, 1};
    double Rx[9] = {1, 0, 0, 0, cos(x), -sin(x), 0, sin(x), cos(x)};
    double Rz2[9] = {cos(z2), -sin(z2), 0, sin(z2), cos(z2), 0, 0, 0, 1};
    double Rz1Rx[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                Rz1Rx[i*3+j] += Rz1[i*3+k]*Rx[k*3+j];
            }
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            mat[i*3+j] = 0;
            for(int k = 0; k < 3; k++){
                mat[i*3+j] += Rz1Rx[i*3+k]*Rz2[k*3+j];
            }
        }
    }
}

void RobotArm::mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    double temp;
    for(uint i = 0; i < row_1; i++){
        for(uint j = 0; j < col_2; j++){
            temp = 0;
            for(uint k = 0; k < row_2; k++){
                temp += mat_1[i*col_1 + k]*mat_2[k*col_2 + j];
            }
            mat_3[i*col_2 + j] = temp;
        }
    }
}

void RobotArm::mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    double temp;
    for(uint i = 0; i < row_1; i++){
        temp = 0;
        for(uint j = 0; j < row_2; j++){
            temp += mat_1[i*col_1 + j]*vec_2[j];
        }
        vec_3[i] = temp;
    }
}

void RobotArm::rpy2mat(double yaw, double pitch, double roll, double *mat)
{
    double R_yaw[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1};
    double R_pitch[9] = {cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch)};
    double R_roll[9] = {1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll)};
    double R_yaw_R_pitch[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                R_yaw_R_pitch[i*3+j] += R_yaw[i*3+k]*R_pitch[k*3+j];
            }
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            mat[i*3+j] = 0;
            for(int k = 0; k < 3; k++){
                mat[i*3+j] += R_yaw_R_pitch[i*3+k]*R_roll[k*3+j];
            }
        }
    }
}

void RobotArm::mat_to_axis_angle(double R_init[], double R_final[], double r[], double *theta)
{
    double R[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                R[i*3+j] += R_init[k*3+i]*R_final[k*3+j];
            }
        }
    }

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = R[0*3 + 0]; m01 = R[0*3 + 1]; m02 = R[0*3 + 2];
    m10 = R[1*3 + 0]; m11 = R[1*3 + 1]; m12 = R[1*3 + 2];
    m20 = R[2*3 + 0]; m21 = R[2*3 + 1]; m22 = R[2*3 + 2];

    double angle, x, y, z; // variables for result
    double epsilon = 0.01; // margin to allow for rounding errors
    double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

    if (abs(m01 - m10) < epsilon && abs(m02 - m20) < epsilon && abs(m12 - m21) < epsilon){
        // singularity found
        // first check for identity matrix which must have +1 for all terms
        // in leading diagonal and zero in other terms
        if (abs(m01 + m10) < epsilon2 && abs(m02 + m20) < epsilon2 && abs(m12 + m21) < epsilon2 && abs(m00 + m11 + m22 - 3) < epsilon2){
            angle = 0;
            x = 1;
            y = 0;
            z = 0;
        }
        else{
            angle = M_PI;
            double xx = (m00 + 1)/2;
            double yy = (m11 + 1)/2;
            double zz = (m22 + 1)/2;
            double xy = (m01 + m10)/4;
            double xz = (m02 + m20)/4;
            double yz = (m12 + m21)/4;
            if (xx > yy && xx > zz){ // m00 is the largest diagonal term
                if (xx < epsilon){
                    x = 0;
                    y = 0.7071;
                    z = 0.7071;
                }
                else{
                    x = sqrt(xx);
                    y = xy/x;
                    z = xz/x;
                }
            }
            else if(yy > zz){ // m11 is the largest diagonal term
                if (yy < epsilon){
                    x = 0.7071;
                    y = 0;
                    z = 0.7071;
                }
                else{
                    y = sqrt(yy);
                    x = xy/y;
                    z = yz/y;
                }
            }
            else{ // m22 is the largest diagonal term so base result on this
                if (zz < epsilon){
                    x = 0.7071;
                    y = 0.7071;
                    z = 0;
                }
                else{
                    z = sqrt(zz);
                    x = xz/z;
                    y = yz/z;
                }
            }
        }
    }
    else {
        // as we have reached here there are no signularites so we can handle normally
        double s = sqrt((m21 - m12)*(m21 - m12) + (m02 - m20)*(m02 - m20) + (m10 - m01)*(m10 - m01));
        if (abs(s) < 0.001){
            s = 1;
        }
        // prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caought by singularity test above, but I've left it in just in case
        angle = acos((m00 + m11 + m22 - 1)/2);
        x = (m21 - m12)/s;
        y = (m02 - m20)/s;
        z = (m10 - m01)/s;
    }

    *theta = angle;
    r[0] = x;
    r[1] = y;
    r[2] = z;
}

void RobotArm::axis_angle_to_mat(double r[], double angle, double mat[])
{
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    double x = r[0], y = r[1], z = r[2];
//      if axis is not already normalised then uncomment this
    double magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude==0) {
        return;
    }
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = c + x*x*t;
    m11 = c + y*y*t;
    m22 = c + z*z*t;

    double tmp1 = x*y*t;
    double tmp2 = z*s;
    m10 = tmp1 + tmp2;
    m01 = tmp1 - tmp2;
    tmp1 = x*z*t;
    tmp2 = y*s;
    m20 = tmp1 - tmp2;
    m02 = tmp1 + tmp2;
    tmp1 = y*z*t;
    tmp2 = x*s;
    m21 = tmp1 + tmp2;
    m12 = tmp1 - tmp2;

    mat[0*3 + 0] = m00; mat[0*3 + 1] = m01; mat[0*3 + 2] = m02;
    mat[1*3 + 0] = m10; mat[1*3 + 1] = m11; mat[1*3 + 2] = m12;
    mat[2*3 + 0] = m20; mat[2*3 + 1] = m21; mat[2*3 + 2] = m22;
}

void RobotArm::mat2rpy(double mat[], double ori[])
{
    ori[0] = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
    ori[1] = atan2(-mat[2 * 3 + 0], sqrt(pow(mat[2 * 3 + 1], 2.0) + pow(mat[2 * 3 + 2], 2.0)));
    ori[2] = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
}

RobotArm::RobotArm(uint numbody, uint DOF, double step_size) {
    num_body = numbody;
    dof = DOF;

    PH = new double[dof];
    PH_pos = new double[3 * num_body];
    PH_ori = new double[3 * num_body];
    delta_q = new double[dof];
    J = new double[num_body * dof];
    JD = new double[dof * num_body];

    M = new double[num_body*num_body];
    Q = new double[num_body];
    Q_c = new double[num_body];
    Q_g = new double[num_body];

    body = new Body[num_body+1];

    lamda = 0.0001;

    // read data
    start_time = 0;
    h = step_size;
    g = -9.80665;

#if MODULE_TYPE == 1
    // DH paramter
    // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
    // |=====================================================|
    // |  1   |    -90     |  0      |   0      |     90     |
	// |  2   |     0      |  151.75 |   0      |    -90     |
	// |  3   |     0      |  150    |   0      |     0      |
	// |  4   |     90     |  86.75  |   0      |     90     |
	// |  5   |     90     |  0      |   -20.25 |     90     |
	// |  6   |     0      |  0      |   102.5  |     0      |

    DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = 90;
    DH[4] = 0;      DH[5] = 0.15175;    DH[6] = 0;          DH[7] = -90;
    DH[8] = 0;      DH[9] = 0.150;      DH[10] = 0;         DH[11] = 0;
    DH[12] = 90;    DH[13] = 0.08675;   DH[14] = 0;         DH[15] = 90;
    DH[16] = 90;    DH[17] = 0;         DH[18] = -0.02025;  DH[19] = 90;
    DH[20] = 0;     DH[21] = 0;         DH[22] = 0.1025;    DH[23] = 0;

    // body 0 variable
    body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
    body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
    body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

    body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

    // preliminary work
    memset(body[0].Yih, 0, sizeof(double)*6);
    memset(body[0].wi, 0, sizeof(double)*3);
    memset(body[0].wit, 0, sizeof(double)*9);

    Body::ang2mat(0, 0, 0, body[0].Cij);
    body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

    body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
    body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

    body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

    // body 1 variables
    Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
    body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, M_PI_2, 0, body[1].Cii, false);
    body[1].rhoip[0] = -0.00189888; body[1].rhoip[1] = -1.44683e-8; body[1].rhoip[2] = -0.0234351;
    body[1].mi = 6.33612131907843e-002;
    body[1].Ixx = 5.46760988093101e-005;
    body[1].Iyy = 4.11897872591055e-005;
    body[1].Izz = 2.28294446378339e-005;
    body[1].Ixy = 1.16933891602143e-011;
    body[1].Iyz = 1.72355337398552e-006;
    body[1].Izx = 3.03099214889948e-011;
    body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
    body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
    body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
    body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

    // body 2 variables
    Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
    body[2].sijp[0] = 0; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

    Body::ang2mat(M_PI, 0, 0, body[2].Cii, false);
    body[2].rhoip[0] = -0.000462227; body[2].rhoip[1] = -0.0427355; body[2].rhoip[2] = 0.000759913;
    body[2].mi = 0.291144481135948;
    body[2].Ixx = 6.84357146533933e-004;
    body[2].Iyy = 1.19767708650701e-004;
    body[2].Izz = 6.2201207394514e-004;
    body[2].Ixy = -7.26891485430593e-006;
    body[2].Iyz = 5.05996626479478e-006;
    body[2].Izx = 1.80750423403909e-007;
    body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
    body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
    body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
    body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

    // body 3 variables
    Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
    body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, 0, 0, body[3].Cii, false);
    body[3].rhoip[0] = 0.075; body[3].rhoip[1] = 0; body[3].rhoip[2] = 0.000807364;
    body[3].mi = 0.416638668104345;
    body[3].Ixx = 1.45776042402133e-003;
    body[3].Iyy = 1.15949266176089e-004;
    body[3].Izz = 1.44207442743259e-003;
    body[3].Ixy = -2.14630188922107e-014;
    body[3].Iyz = -4.86620428197596e-019;
    body[3].Izx = -5.85663447574856e-020;
    body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
    body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
    body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
    body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

    // body 4 variables
    Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
    body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, 0, 0, body[4].Cii, false);
    body[4].rhoip[0] = 0.000749752; body[4].rhoip[1] = 0.0609445; body[4].rhoip[2] = 0.000415268;
    body[4].mi = 0.228993914748238;
    body[4].Ixx = 7.74704754240776e-005;
    body[4].Iyy = 2.01161940464821e-004;
    body[4].Izz = 1.88922599914013e-004;
    body[4].Ixy = 2.91568180110741e-006;
    body[4].Iyz = 7.12741866241557e-008;
    body[4].Izx = -2.20466982640091e-006;
    body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
    body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
    body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
    body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

    // body 5 variables
    Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
    body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

    Body::ang2mat(M_PI, M_PI_2, 0, body[5].Cii, false);
    body[5].rhoip[0] = 0.0555687; body[5].rhoip[1] = 0; body[5].rhoip[2] = -0.000237633;
    body[5].mi = 0.204137411295743;
    body[5].Ixx = 9.4696526893192e-005;
    body[5].Iyy = 7.92107777080459e-005;
    body[5].Izz = 1.38821213983018e-004;
    body[5].Ixy = -2.02238967554624e-005;
    body[5].Iyz = -1.20999283701959e-015;
    body[5].Izx = 1.87131808263915e-015;
    body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
    body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
    body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
    body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

    // body 6 variables
    Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
    body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

    Body::ang2mat(M_PI_2, M_PI_2, -M_PI_2, body[6].Cii, false);
    body[6].rhoip[0] = 5.39394e-10; body[6].rhoip[1] = 3.33671e-8; body[6].rhoip[2] = 0.089384;
    body[6].mi = 2.08363885223627e-002;
    body[6].Ixx = 2.66302463617021e-006;
    body[6].Iyy = 1.56637607668211e-006;
    body[6].Izz = 1.88187616526518e-006;
    body[6].Ixy = 2.4095425326714e-012;
    body[6].Iyz = 2.738802635816e-013;
    body[6].Izx = 9.27461478843821e-014;
    body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
    body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
    body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
    body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
#elif MODULE_TYPE == 2
	// DH paramter
	// | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
	// |=====================================================|
	// |  1   |    -90     |  0      |   0      |    -90     |
	// |  2   |     0      |  164.25 |   16.50  |    -90     |
	// |  3   |     180    |  170    |   0      |     0      |
	// |  4   |     90     |  65.25  |   0      |     90     |
	// |  5   |     90     |  0      |  -16.75  |     90     |
	// |  6   |     0      |  0      |   84     |     0      |

	DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = -90;
    DH[4] = 0;      DH[5] = 0.16425;    DH[6] = -0.0165;     DH[7] = -90;
	DH[8] = 180;    DH[9] = 0.170;      DH[10] = 0;         DH[11] = 0;
	DH[12] = 90;    DH[13] = 0.06525;   DH[14] = 0;         DH[15] = 90;
	DH[16] = 90;    DH[17] = 0;         DH[18] = -0.01675;  DH[19] = 90;
	DH[20] = 0;     DH[21] = 0;         DH[22] = 0.084;     DH[23] = 0;

	// body 0 variable
	body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
	body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
	body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

	body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

	// preliminary work
	memset(body[0].Yih, 0, sizeof(double)*6);
	memset(body[0].wi, 0, sizeof(double)*3);
	memset(body[0].wit, 0, sizeof(double)*9);

	Body::ang2mat(0, 0, 0, body[0].Cij);
	body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

	body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
	body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

	body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

	// body 1 variables
	Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
	body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

	Body::ang2mat(0, 0, 0, body[1].Cii, false);
	body[1].rhoip[0] = -0.000135874; body[1].rhoip[1] = -1.068e-12; body[1].rhoip[2] = -0.0246531;
	body[1].mi = 7.08684533226054e-002;
	body[1].Ixx = 2.15089916203442e-005;
	body[1].Iyy = 3.65070109080493e-005;
	body[1].Izz = 2.83989018839352e-005;
	body[1].Ixy = -1.24014643551512e-017;
	body[1].Iyz = -1.12222253664962e-010;
	body[1].Izx = 3.33296217281281e-008;
	body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
	body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
	body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
	body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

	// body 2 variables
	Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
	body[2].sijp[0] = DH[1*4+2]; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

	Body::ang2mat(0, M_PI_2, M_PI_2, body[2].Cii, false);
	body[2].rhoip[0] = 0.00264336; body[2].rhoip[1] = -0.0319009; body[2].rhoip[2] = -0.000524792;
	body[2].mi = 0.233270004294732;
	body[2].Ixx = 4.08019849963512e-004;
	body[2].Iyy = 4.36730722674176e-004;
	body[2].Izz = 8.71040040349363e-005;
	body[2].Ixy = -3.23596533711006e-007;
	body[2].Iyz = -4.58645443586337e-005;
	body[2].Izx = -2.40916440855742e-006;
	body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
	body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
	body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
	body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

	// body 3 variables
	Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
	body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

	Body::ang2mat(M_PI_2,M_PI_2,M_PI_2, body[3].Cii, false);
	body[3].rhoip[0] = 0.0668431; body[3].rhoip[1] = -4.49044e-11; body[3].rhoip[2] = -0.000574255;
	body[3].mi = 0.294733648136712;
	body[3].Ixx = 1.33438729955757e-003;
	body[3].Iyy = 1.35236609017727e-003;
	body[3].Izz = 6.10851857303522e-005;
	body[3].Ixy = -1.65934500194573e-013;
	body[3].Iyz = 5.8944693629749e-013;
	body[3].Izx = -1.46477397988517e-006;
	body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
	body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
	body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
	body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

	// body 4 variables
	Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
	body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

	Body::ang2mat(M_PI_2,M_PI_2,-M_PI_2, body[4].Cii, false);
	body[4].rhoip[0] = 0.000488263; body[4].rhoip[1] = 0.0465912; body[4].rhoip[2] = 3.24848e-5;
	body[4].mi = 0.108749563323237;
	body[4].Ixx = 5.19451711277109e-005;
	body[4].Iyy = 2.17677195227188e-005;
	body[4].Izz = 5.3423100843467e-005;
	body[4].Ixy = -5.50565486810879e-008;
	body[4].Iyz = -4.1508751024039e-007;
	body[4].Izx = -1.72489029231613e-009;
	body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
	body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
	body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
	body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

	// body 5 variables
	Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
	body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

	Body::ang2mat(-M_PI_2, 0, 0, body[5].Cii, false);
	body[5].rhoip[0] = 0.0449512; body[5].rhoip[1] = -1.30501e-12; body[5].rhoip[2] = -0.00250684;
	body[5].mi = 0.110204790536652;
	body[5].Ixx = 5.65872649539517e-005;
	body[5].Iyy = 3.21370607982722e-005;
	body[5].Izz = 3.83261110287993e-005;
	body[5].Ixy = 3.11849020965568e-015;
	body[5].Iyz = 2.2933899377825e-006;
	body[5].Izx = 1.98601435820104e-015;
	body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
	body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
	body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
	body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

	// body 6 variables
	Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
	body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

	Body::ang2mat(M_PI, M_PI_2, 0, body[6].Cii, false);
	body[6].rhoip[0] = 4.25231e-8; body[6].rhoip[1] = 0.00049999; body[6].rhoip[2] = 0.0756101;
	body[6].mi = 1.06900256777816e-002;
	body[6].Ixx = 5.4230201398644e-007;
	body[6].Iyy = 8.27349038593228e-007;
	body[6].Izz = 6.80541319418483e-007;
	body[6].Ixy = 1.68734917731254e-012;
	body[6].Iyz = 1.25961065157397e-013;
	body[6].Izx = 3.11703037783589e-013;
	body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
	body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
	body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
	body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
#endif

    numeric = new Numerical();
}

RobotArm::~RobotArm() {
    delete[] PH;
    delete[] PH_pos;
    delete[] PH_ori;
    delete[] delta_q;
    delete[] J;
    delete[] JD;

    delete[] M;
    delete[] Q;
    delete[] Q_c;
    delete[] Q_g;

    delete[] body;
    delete numeric;
}

#ifdef FILEIO_H_
void RobotArm::run_kinematics()
{
    sprintf(file_name, "../FAR_Analysis/data/evaluation_motion_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");

    for(uint indx = 0; indx < row; indx++) {
        for (uint i = 1; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
        }

        kinematics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::run_kinematics(double *q, double *des_pose){
    for(int i = 0; i < 6; i++){
        body[i+1].qi = q[i];
    }

    kinematics();

    des_pose[0] = body[6].re[0];
    des_pose[1] = body[6].re[1];
    des_pose[2] = body[6].re[2];
    des_pose[3] = body[6].ori[0];
    des_pose[4] = body[6].ori[1];
    des_pose[5] = body[6].ori[2];
}

#ifdef FILEIO_H_
void RobotArm::run_inverse_kinematics() {
    sprintf(file_name, "../FAR_Analysis/data/evaluation_motion_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");
    for (uint i = 1; i <= 6; i++) {
        body[i].qi = ref_data[0*col + i + 1];
    }

    double pos_d[3], ori_d[3];
    for (uint indx = 0; indx < row; indx++) {
        pos_d[0] = ref_data[indx*col + 8];
        pos_d[1] = ref_data[indx*col + 9];
        pos_d[2] = ref_data[indx*col + 10];
        ori_d[0] = ref_data[indx*col + 11];
        ori_d[1] = ref_data[indx*col + 12];
        ori_d[2] = ref_data[indx*col + 13];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::run_inverse_kinematics(double* input_q, double* des_pose, double* cur_joint, double* cur_pose){
    bool goal_reach = false;

    double epsilon_pos = 0.05;
    double epsilon_ang = 1;

    for (uint i = 1; i <= num_body; i++) {
        body[i].qi = input_q[i - 1];
    }

    double pos_d[3], ori_d[3];

//    for(uint i = 0; i < 5; i++){
        pos_d[0] = des_pose[0];
        pos_d[1] = des_pose[1];
        pos_d[2] = des_pose[2];
        ori_d[0] = des_pose[3];
        ori_d[1] = des_pose[4];
        ori_d[2] = des_pose[5];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        for(uint i = 1; i <= num_body; i++){
            cur_joint[i - 1] = body[i].qi;
        }

//        kinematics();

        cur_pose[0] = body[num_body].re[0];
        cur_pose[1] = body[num_body].re[1];
        cur_pose[2] = body[num_body].re[2];
        cur_pose[3] = body[num_body].ori[0];
        cur_pose[4] = body[num_body].ori[1];
        cur_pose[5] = body[num_body].ori[2];

//        double pos = sqrt(pow(des_pose[0] - cur_pose[0], 2) + pow(des_pose[1] - cur_pose[1], 2) + pow(des_pose[2] - cur_pose[2], 2));
//        double ang_r = abs(des_pose[3] - cur_pose[3]);
//        double ang_p = abs(des_pose[4] - cur_pose[4]);
//        double ang_y = abs(des_pose[5] - cur_pose[5]);

//        printf("[IK]pos : %f\n", pos);
//        printf("[IK]ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

//        if (pos < epsilon_pos && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang){
//            goal_reach = true;
////            printf("[IK]iteration : %d\n", i);
//            break;
//        }
//        else{
//            goal_reach = false;
//        }
//    }
}

#ifdef FILEIO_H_
void RobotArm::run_dynamics(){
    sprintf(file_name, "../FAR_Analysis/data/evaluation_dynamics_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 20;
    load_data("../FAR_Analysis/data/evaluation_dynamics_recurdyn.txt", &ref_data, "\t");

    for (uint indx = 0; indx < row; indx++) {
        for (uint i = 0; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
            body[i].qi_dot = ref_data[indx*col + i + 1 + 6];
        }

        kinematics();
        dynamics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::gravity_compensation(double *q, double *q_dot, double *torque){
    for(uint i = 1; i <= num_body; i++){
        body[i].qi = q[i - 1];
        body[i].qi_dot = q_dot[i - 1];
    }

    kinematics();
    dynamics();

    for(uint i = 0; i < 6; i++){
        torque[i] = -Q[i];
    }
}

void RobotArm::kinematics()
{
    Body *body0, *body1;
    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        // orientation
        double *Aijpp_ptr = body1->Aijpp;
        *(Aijpp_ptr++) = cos(body1->qi);	*(Aijpp_ptr++) = -sin(body1->qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = sin(body1->qi);	*(Aijpp_ptr++) = cos(body1->qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = 0;					*(Aijpp_ptr++) = 0;					*(Aijpp_ptr++) = 1;

        mat(body0->Ai, body0->Cij, 3, 3, 3, 3, body1->Ai_Cij);
        mat(body1->Ai_Cij, body1->Aijpp, 3, 3, 3, 3, body1->Ai);
        mat(body1->Ai_Cij, body1->u_vec, 3, 3, 3, body1->zi);

        // position
        mat(body0->Ai, body0->sijp, 3, 3, 3, body0->sij);
        for (uint i = 0; i < 3; i++){
            body1->ri[i] = body0->ri[i] + body0->sij[i];
        }
    }

    // End point
    Body *body_end = &body[num_body];
    mat(body_end->Ai, body_end->sijp, 3, 3, 3, body_end->sij);
    for (uint i = 0; i < 3; i++){
        body_end->re[i] = body_end->ri[i] + body_end->sij[i];
    }

    mat(body_end->Ai, body_end->Cij, 3, 3, 3, 3, body_end->Ae);

    mat2rpy(body_end->Ae, body_end->ori);

//    body_end->ori[0] = atan2(body_end->Ae[2 * 3 + 1], body_end->Ae[2 * 3 + 2]);
//    body_end->ori[1] = atan2(-body_end->Ae[2 * 3 + 0], sqrt(pow(body_end->Ae[2 * 3 + 1], 2.0) + pow(body_end->Ae[2 * 3 + 2], 2.0)));
//    body_end->ori[2] = atan2(body_end->Ae[1 * 3 + 0], body_end->Ae[0 * 3 + 0]);
}

void RobotArm::inverse_kinematics(double des_pos[3], double des_ang[3]) {
    int *indx = new int[6];
    double *fac = new double[36];

    double desired[6] = {0,};
    for(int i = 0; i < 3; i++){
        desired[i] = des_pos[i];
        desired[i + 3] = des_ang[i];
    }

    double alpha = 1/4.0;
    double err[6], qdot[6];
    for(int i = 0; i < 1/alpha; i++){
        for(int j = 0; j < 3; j++){
            err[j] = des_pos[j] - body[num_body].re[j];
            err[j + 3] = des_ang[j] - body[num_body].ori[j];
        }

        jacobian();

        numeric->ludcmp(J, 6, indx, 0.0, fac);
        numeric->lubksb(fac, 6, indx, err, qdot);

        for (uint j = 0; j < 6; j++){
            delta_q[j] = qdot[j]*alpha;
        }

        for (uint j = 0; j < num_body; j++) {
            body[j + 1].qi += delta_q[j];
        }

        kinematics();
    }

    delete[] indx;
    delete[] fac;


//    Body *body_end = &body[num_body];
//    for (uint i = 0; i < 3; i++) {
//        PH_pos[i] = des_pos[i] - body_end->re[i];
//        PH_ori[i] = des_ang[i] - body_end->ori[i];
//    }

//    for (uint i = 0; i < 3; i++) {
//        PH[i] = PH_pos[i];
//        PH[i + 3] = PH_ori[i];
//    }

//#if 0
//    jacobian();

//    double *U, *s, *V;
//    U = new double[dof * dof];
//    s = new double[MIN(dof, num_body)];
//    V = new double[num_body*num_body];

//    numeric->svdcmp(J, dof, num_body, U, s, V);

//    memset(JD, 0, sizeof(double) * num_body*dof);
//    double *temp = new double[num_body*dof];
//    double lamda = 1e-5;
//    for (uint i = 0; i < dof; i++) {
//        for (uint j = 0; j < num_body; j++) {
//            for (uint k = 0; k < dof; k++) {
//                temp[j * dof + k] = V[j * num_body + i] * U[k * num_body + i];
//            }
//        }
//        for (uint j = 0; j < num_body; j++) {
//            for (uint k = 0; k < dof; k++) {
//                JD[j * dof + k] += (s[i] / (s[i]*s[i] +lamda*lamda))*(temp[j * dof + k]);
//            }
//        }
//    }

//    delete[] s;
//    delete[] U;
//    delete[] V;
//    delete[] temp;


//    memset(delta_q, 0, sizeof(double) * 6);
//    for (uint i = 0; i < num_body; i++) {
//        for (uint j = 0; j < num_body; j++) {
//            delta_q[i] += JD[i * num_body + j] * PH[j];
//        }
//    }
//#else

//    int *indx = new int[6];
//    double *fac = new double[6*6];
//    double errmax = 0;
//    int NRcount = 0;

//    do{
//        jacobian();

//        numeric->ludcmp(J, 6, indx, 0.0, fac);
//        memset(delta_q, 0, sizeof(double) * 6);
//        numeric->lubksb(fac, 6, indx, PH, delta_q);

//        for (uint i = 0; i < num_body; i++) {
//            body[i + 1].qi += delta_q[i];
//        }

//        kinematics();

//        for (uint i = 0; i < 3; i++) {
//            PH_pos[i] = des_pos[i] - body_end->re[i];
//            PH_ori[i] = des_ang[i] - body_end->ori[i];
//        }

//        for (uint i = 0; i < 3; i++) {
//            PH[i] = PH_pos[i];
//            PH[i + 3] = PH_ori[i];
//        }

//        errmax = PH[0];
//        for(uint i = 1; i < num_body;i++){
//            errmax = errmax > abs(PH[i]) ? errmax : abs(PH[i]);
//        }

//        NRcount++;
//    }while(errmax > 1e-3 && NRcount < 10);

////    rt_printf("[IK]Err Max : %E\t : Iteration : %d\n", errmax, NRcount);

//    delete[] indx;
//    delete[] fac;
//#endif
}

void RobotArm::jacobian()
{
    double *Jv = new double[3 * num_body];
    double *Jw = new double[3 * num_body];

    Body *body0, *body1;
    Body *body_end = &body[num_body];

#if 0
//    A[9] = {cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI), 0, 0, 0, 1};
    A[0] = cos(M_PI);
    A[1] = -sin(M_PI);
    A[2] = 0;
    A[3] = sin(M_PI);
    A[4] = cos(M_PI);
    A[5] = 0;
    A[6] = 0;
    A[7] = 0;
    A[8] = 1;

    for (uint indx = 1; indx <= num_body; indx++){
        body0 = &body[indx - 1];
        body1 = &body[indx];

        for(uint i = 0; i < 3; i++){
            body1->oi[i] = body_end->re[i] - body1->ri[i];
        }
        tilde(body1->zi, body1->zit);
        mat(body1->zit, body1->oi, 3, 3, 3, body1->Jvi);
        mat(A, body1->zi, 3, 3, 3, body1->Jwi);

        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + (indx - 1)] = body1->Jvi[i];
            Jw[i*num_body + (indx - 1)] = body1->Jwi[i];
        }
    }
#elif 1
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body1->Aijpp_qi[0] = -sin(body1->qi); body1->Aijpp_qi[1] = -cos(body1->qi); body1->Aijpp_qi[2] = 0;
        body1->Aijpp_qi[3] =  cos(body1->qi); body1->Aijpp_qi[4] = -sin(body1->qi); body1->Aijpp_qi[5] = 0;
        body1->Aijpp_qi[6] = 0; body1->Aijpp_qi[7] = 0; body1->Aijpp_qi[8] = 0;
    }

    for(uint indx = 1; indx <= num_body; indx++){
        memset(body[indx].Ae_qi, 0, sizeof(double)*9);
        memset(body[indx].re_qi, 0, sizeof(double)*3);
        for(uint indx2 = indx; indx2 <= num_body; indx2++){
            if (indx == indx2){
                mat(body[indx2].Ai_Cij, body[indx2].Aijpp_qi, 3, 3, 3, 3, body[indx2].Ai_Cij_Aijpp_qi);
                for(uint i = 0; i < 9; i++){
                    body[indx].Ae_qi[i] += body[indx2].Ai_Cij_Aijpp_qi[i];
                }
            }
            else{
                mat(body[indx].Ae_qi, body[indx2 - 1].Cij, 3, 3, 3, 3, body[indx2].Ae_qi_Cij);
                mat(body[indx2].Ae_qi_Cij, body[indx2].Aijpp, 3, 3, 3, 3, body[indx].Ae_qi_Cij_Aijpp);
                memcpy(body[indx].Ae_qi, body[indx].Ae_qi_Cij_Aijpp, sizeof(double)*9);
            }
            if (indx2 < num_body){
                mat(body[indx].Ae_qi, body[indx2].sijp, 3, 3, 3, body[indx2].Ae_qi_sijp);
                for(uint i = 0; i < 3; i++){
                    body[indx].re_qi[i] += body[indx2].Ae_qi_sijp[i];
                }
            }
        }
    }

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ae_qi, body_end->sijp, 3, 3, 3, body1->Ae_qi_end);
        if (indx < num_body){
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->re_qi[i] + body1->Ae_qi_end[i];
            }
        }
        else{
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->Ae_qi_end[i];
            }
        }
        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body1->Jvi[i];
        }
    }

    Ae_31 = body_end->Ae[6];
    Ae_32 = body_end->Ae[7];
    Ae_33 = body_end->Ae[8];
    Ae_21 = body_end->Ae[3];
    Ae_11 = body_end->Ae[0];

    Ae_32_33 = Ae_32/Ae_33;
    body[1].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[1].roll_qi_2 = ((body[1].Ae_qi[7]*Ae_33 - Ae_32*body[1].Ae_qi[8])/(pow(Ae_33, 2))); body[1].roll_qi = body[1].roll_qi_1*body[1].roll_qi_2;
    body[2].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[2].roll_qi_2 = ((body[2].Ae_qi[7]*Ae_33 - Ae_32*body[2].Ae_qi[8])/(pow(Ae_33, 2))); body[2].roll_qi = body[2].roll_qi_1*body[2].roll_qi_2;
    body[3].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[3].roll_qi_2 = ((body[3].Ae_qi[7]*Ae_33 - Ae_32*body[3].Ae_qi[8])/(pow(Ae_33, 2))); body[3].roll_qi = body[3].roll_qi_1*body[3].roll_qi_2;
    body[4].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[4].roll_qi_2 = ((body[4].Ae_qi[7]*Ae_33 - Ae_32*body[4].Ae_qi[8])/(pow(Ae_33, 2))); body[4].roll_qi = body[4].roll_qi_1*body[4].roll_qi_2;
    body[5].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[5].roll_qi_2 = ((body[5].Ae_qi[7]*Ae_33 - Ae_32*body[5].Ae_qi[8])/(pow(Ae_33, 2))); body[5].roll_qi = body[5].roll_qi_1*body[5].roll_qi_2;
    body[6].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[6].roll_qi_2 = ((body[6].Ae_qi[7]*Ae_33 - Ae_32*body[6].Ae_qi[8])/(pow(Ae_33, 2))); body[6].roll_qi = body[6].roll_qi_1*body[6].roll_qi_2;

    Ae_32_33_2 = pow(Ae_32, 2) + pow(Ae_33, 2);
    body[1].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[1].pitch_qi_2 = (-body[1].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[1].Ae_qi[8] + Ae_33*body[1].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[1].pitch_qi = body[1].pitch_qi_1*body[1].pitch_qi_2;
    body[2].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[2].pitch_qi_2 = (-body[2].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[2].Ae_qi[8] + Ae_33*body[2].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[2].pitch_qi = body[2].pitch_qi_1*body[2].pitch_qi_2;
    body[3].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[3].pitch_qi_2 = (-body[3].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[3].Ae_qi[8] + Ae_33*body[3].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[3].pitch_qi = body[3].pitch_qi_1*body[3].pitch_qi_2;
    body[4].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[4].pitch_qi_2 = (-body[4].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[4].Ae_qi[8] + Ae_33*body[4].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[4].pitch_qi = body[4].pitch_qi_1*body[4].pitch_qi_2;
    body[5].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[5].pitch_qi_2 = (-body[5].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[5].Ae_qi[8] + Ae_33*body[5].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[5].pitch_qi = body[5].pitch_qi_1*body[5].pitch_qi_2;
    body[6].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[6].pitch_qi_2 = (-body[6].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[6].Ae_qi[8] + Ae_33*body[6].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[6].pitch_qi = body[6].pitch_qi_1*body[6].pitch_qi_2;

    Ae_21_11 = Ae_21/Ae_11;
    body[1].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[1].yaw_qi_2 = ((body[1].Ae_qi[3]*Ae_11 - Ae_21*body[1].Ae_qi[0])/(pow(Ae_11, 2))); body[1].yaw_qi = body[1].yaw_qi_1*body[1].yaw_qi_2;
    body[2].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[2].yaw_qi_2 = ((body[2].Ae_qi[3]*Ae_11 - Ae_21*body[2].Ae_qi[0])/(pow(Ae_11, 2))); body[2].yaw_qi = body[2].yaw_qi_1*body[2].yaw_qi_2;
    body[3].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[3].yaw_qi_2 = ((body[3].Ae_qi[3]*Ae_11 - Ae_21*body[3].Ae_qi[0])/(pow(Ae_11, 2))); body[3].yaw_qi = body[3].yaw_qi_1*body[3].yaw_qi_2;
    body[4].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[4].yaw_qi_2 = ((body[4].Ae_qi[3]*Ae_11 - Ae_21*body[4].Ae_qi[0])/(pow(Ae_11, 2))); body[4].yaw_qi = body[4].yaw_qi_1*body[4].yaw_qi_2;
    body[5].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[5].yaw_qi_2 = ((body[5].Ae_qi[3]*Ae_11 - Ae_21*body[5].Ae_qi[0])/(pow(Ae_11, 2))); body[5].yaw_qi = body[5].yaw_qi_1*body[5].yaw_qi_2;
    body[6].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[6].yaw_qi_2 = ((body[6].Ae_qi[3]*Ae_11 - Ae_21*body[6].Ae_qi[0])/(pow(Ae_11, 2))); body[6].yaw_qi = body[6].yaw_qi_1*body[6].yaw_qi_2;

    body[1].Jwi[0] = body[1].roll_qi; body[1].Jwi[1] = body[1].pitch_qi; body[1].Jwi[2] = body[1].yaw_qi;
    body[2].Jwi[0] = body[2].roll_qi; body[2].Jwi[1] = body[2].pitch_qi; body[2].Jwi[2] = body[2].yaw_qi;
    body[3].Jwi[0] = body[3].roll_qi; body[3].Jwi[1] = body[3].pitch_qi; body[3].Jwi[2] = body[3].yaw_qi;
    body[4].Jwi[0] = body[4].roll_qi; body[4].Jwi[1] = body[4].pitch_qi; body[4].Jwi[2] = body[4].yaw_qi;
    body[5].Jwi[0] = body[5].roll_qi; body[5].Jwi[1] = body[5].pitch_qi; body[5].Jwi[2] = body[5].yaw_qi;
    body[6].Jwi[0] = body[6].roll_qi; body[6].Jwi[1] = body[6].pitch_qi; body[6].Jwi[2] = body[6].yaw_qi;

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        for (uint i = 0; i < 3; i++){
            Jw[i*num_body + indx - 1] = body1->Jwi[i];
        }
    }


#else
    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        mat(body0->Cij, body1->Aijpp, 3, 3, 3, 3, body0->Cij_Aijpp);

        double *Aijpp_qi_ptr = body1->Aijpp_qi;
        *(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = -cos(body1->qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) =  cos(body1->qi);	*(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr) = 0;

        mat(body1->Ai_Cij, body1->Aijpp_qi, 3, 3, 3, 3, body1->Ai_Cij_Aijpp_qi);
    }

    double temp1[9] = { 0, }, temp2[3] = {0,};
    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        memset(body1->A6_qi, 0, sizeof(double) * 9);
        memset(body1->r6_qi, 0, sizeof(double) * 3);
        for (uint indx2 = indx; indx2 <= num_body; indx2++) {
            if (indx2 == indx) {
                for(uint i = 0; i < 9;i++){
                    body1->A6_qi[i] += body1->Ai_Cij_Aijpp_qi[i];
                }
            }
            else {
                mat(body1->A6_qi, body[indx2 - 1].Cij_Aijpp, 3, 3, 3, 3, temp1);
                memcpy(body1->A6_qi, temp1, sizeof(double) * 9);
            }
            if (indx2 < num_body) {
                mat(body1->A6_qi, body[indx2].sijp, 3, 3, 3, temp2);
                for(uint i = 0; i < 3; i++){
                    body1->r6_qi[i] += temp2[i];
                }
            }
        }

        mat(body1->A6_qi, body_end->Cij, 3, 3, 3, 3, body1->Ae_qi);
        mat(body1->A6_qi, body_end->sijp, 3, 3, 3, body1->re_qi);
        for (uint i = 0; i < 3; i++){
            body1->re_qi[i] += body1->r6_qi[i];
        }

        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body1->re_qi[i];
        }
    }

     Ae_31 = body_end->Ae[6];
     Ae_32 = body_end->Ae[7];
     Ae_33 = body_end->Ae[8];
     Ae_21 = body_end->Ae[3];
     Ae_11 = body_end->Ae[0];

     roll_q_temp1 = Ae_32 * Ae_32 + Ae_33 * Ae_33;
     roll_q_temp2 = sqrt(roll_q_temp1);
     roll_q_temp3 = Ae_33 + roll_q_temp2;
     roll_q_temp4 = roll_q_temp2 * (roll_q_temp1 + Ae_33*roll_q_temp2);

     pitch_q_temp1 = sqrt(Ae_32*Ae_32 + Ae_33*Ae_33);
     pitch_q_temp2 = Ae_31 * Ae_31 + pitch_q_temp1 * pitch_q_temp1;
     pitch_q_temp3 = sqrt(pitch_q_temp2);
     pitch_q_temp4 = pitch_q_temp3 * (pitch_q_temp2 + pitch_q_temp1 * pitch_q_temp3);

     yaw_q_temp1 = Ae_21 * Ae_21 + Ae_11 * Ae_11;
     yaw_q_temp2 = sqrt(yaw_q_temp1);
     yaw_q_temp3 = Ae_11 + yaw_q_temp2;
     yaw_q_temp4 = yaw_q_temp2 * (yaw_q_temp1 + Ae_11*yaw_q_temp2);

     for (uint indx = 1; indx <= num_body; indx++) {
         body1 = &body[indx];
         body1->Ae_qi_31 = body1->Ae_qi[6];
         body1->Ae_qi_32 = body1->Ae_qi[7];
         body1->Ae_qi_33 = body1->Ae_qi[8];
         body1->Ae_qi_21 = body1->Ae_qi[3];
         body1->Ae_qi_11 = body1->Ae_qi[0];

         body1->roll_qi = (roll_q_temp3*(body1->Ae_qi_32*Ae_33 - Ae_32*body1->Ae_qi_33)) / roll_q_temp4;
         body1->pitch_qi = -((pitch_q_temp3 + pitch_q_temp1)*(body1->Ae_qi_31*pitch_q_temp1 - Ae_31 * (Ae_32*body1->Ae_qi_32 + Ae_33 * body1->Ae_qi_33)/pitch_q_temp1))/ pitch_q_temp4;
         body1->yaw_qi = (yaw_q_temp3*(body1->Ae_qi_21*Ae_11 - Ae_21*body1->Ae_qi_11)) / yaw_q_temp4;

         Jw[0 * num_body + indx - 1] = body1->roll_qi;
         Jw[1 * num_body + indx - 1] = body1->pitch_qi;
         Jw[2 * num_body + indx - 1] = body1->yaw_qi;
     }
#endif

     memcpy(J, Jv, sizeof(double) * 3 * num_body);
     memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

    delete[] Jv;
    delete[] Jw;
}

void RobotArm::dynamics()
{
    Body *body0, *body1, *body2;
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body0 = &body[indx - 1];
        // velocity state
        mat(body1->Ai_Cij, body1->u_vec, 3, 3, 3, body1->Hi);
        tilde(body1->ri, body1->rit);
        mat(body1->rit, body1->Hi, 3, 3, 3, body1->Bi);
        memcpy(body1->Bi + 3, body1->Hi, sizeof(double)*3);
        for (uint i = 0; i < 6; i++){
            body1->Yih[i] = body0->Yih[i] + body1->Bi[i]*body1->qi_dot;
        }

        // cartesian velocity
        for (uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Ti[i*6 + j] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + (j + 3)] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + j] = 0;
                body1->Ti[i*6 + (j + 3)] = -body1->rit[i*3 + j];
            }
        }

        mat(body1->Ti, body1->Yih, 6, 6, 6, body1->Yib);
        memcpy(body1->ri_dot, body1->Yib, sizeof(double)*3);
        memcpy(body1->wi, body1->Yib + 3, sizeof(double)*3);
        tilde(body1->wi, body1->wit);
        mat(body1->Ai, body1->rhoip, 3, 3, 3, body1->rhoi);
        for(uint i = 0; i < 3; i++){
            body1->ric[i] = body1->ri[i] + body1->rhoi[i];
        }
        mat(body1->wit, body1->rhoi, 3, 3, 3, body1->ric_dot);
        for (uint i = 0; i < 3; i++){
            body1->ric_dot[i] += body1->ri_dot[i];
        }

        // mass & force
        mat(body1->Ai, body1->Cii, 3, 3, 3, 3, body1->Ai_Cii);
        double temp[9] = {0,}, temp2 = 0;
        mat(body1->Ai_Cii, body1->Jip, 3, 3, 3, 3, temp);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                temp2 = 0;
                for(uint k = 0; k < 3; k++){
                    temp2 += temp[i*3 + k]*body1->Ai_Cii[j*3 + k];
                }
                body1->Jic[i*3 + j] = temp2;
            }
        }
        tilde(body1->ri_dot, body1->rit_dot);
        tilde(body1->ric_dot, body1->rict_dot);
        tilde(body1->ric, body1->rict);
        double temp3[9] = {0,};
        mat(body1->rict, body1->rict, 3, 3, 3, 3, temp3);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Mih[i*6 + j] = i == j ? body1->mi : 0;
                body1->Mih[(i + 3)*6 + j] = body1->mi*body1->rict[i*3 + j];
                body1->Mih[i*6 + (j + 3)] = -body1->mi*body1->rict[i*3 + j];
                body1->Mih[(i + 3)*6 + (j + 3)] = body1->Jic[i*3 + j] - body1->mi*temp3[i*3 + j];
            }
        }
        body1->fic[0] = 0;
        body1->fic[1] = 0;
        body1->fic[2] = body1->mi*g;
        body1->tic[0] = 0;
        body1->tic[1] = 0;
        body1->tic[2] = 0;
        double rict_dot_wi[3] = {0,};
        mat(body1->rict_dot, body1->wi, 3, 3, 3, rict_dot_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i] = body1->fic[i];
            body1->Qih_c[i] = body1->mi*rict_dot_wi[i];
            body1->Qih[i] = body1->Qih_g[i] + body1->Qih_c[i];
        }
        double rict_fic[3] = {0,}, rict_rict_dot_wi[3] = {0,}, Jic_wi[3] = {0,}, wit_Jic_wi[3] = {0,};
        mat(body1->rict, body1->fic, 3, 3, 3, rict_fic);
        mat(body1->rict, rict_dot_wi, 3, 3, 3, rict_rict_dot_wi);
        mat(body1->Jic, body1->wi, 3, 3, 3, Jic_wi);
        mat(body1->wit, Jic_wi, 3, 3, 3, wit_Jic_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i + 3] = rict_fic[i];
            body1->Qih_c[i + 3] = body1->mi*rict_rict_dot_wi[i] - wit_Jic_wi[i];
            body1->Qih[i + 3] = body1->tic[i] + body1->Qih_g[i + 3] + body1->Qih_c[i + 3];
        }

        // velocity coupling
        mat(body0->wit, body1->Hi, 3, 3, 3, body1->Hi_dot);
        double rit_dot_Hi[3] = {0,}, rit_Hi_dot[3] = {0,};
        mat(body1->rit_dot, body1->Hi, 3, 3, 3, rit_dot_Hi);
        mat(body1->rit, body1->Hi_dot, 3, 3, 3, rit_Hi_dot);
        for(uint i = 0; i < 3; i++){
            body1->Di[i] = (rit_dot_Hi[i] + rit_Hi_dot[i])*body1->qi_dot;
            body1->Di[i+3] = body1->Hi_dot[i]*body1->qi_dot;
        }

        memcpy(body1->Di_sum, body1->Di, sizeof(double)*6);
        for(uint indx2 = indx - 1; indx2 >= 1; indx2--){
            for(uint i = 0; i < 6; i++){
                body1->Di_sum[i] += body[indx2].Di[i];
            }
        }
    }

    // system EQM
    for(uint i = num_body; i >= 1;  i--){
        body1 = &body[i];
        if (i == num_body){
            memcpy(body1->Ki, body1->Mih, sizeof(double)*36);
            memcpy(body1->Li, body1->Qih, sizeof(double)*6);
            memcpy(body1->Li_g, body1->Qih_g, sizeof(double)*6);
            memcpy(body1->Li_c, body1->Qih_c, sizeof(double)*6);
        }
        else{
            body2 = &body[i + 1];
            for(uint i = 0; i < 36; i++){
                body1->Ki[i] = body1->Mih[i] + body2->Ki[i];
            }
            mat(body2->Ki, body2->Di, 6, 6, 6, body2->Ki_Di);
            for(uint i = 0; i < 6; i++){
                body1->Li[i] = body1->Qih[i] + body2->Li[i] - body2->Ki_Di[i];
                body1->Li_g[i] = body1->Qih_g[i] + body2->Li_g[i] - body2->Ki_Di[i];
                body1->Li_c[i] = body1->Qih_c[i] + body2->Li_c[i] - body2->Ki_Di[i];
            }
        }
    }

    memset(Q, 0, sizeof(double)*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ki, body1->Di_sum, 6, 6, 6, body1->Ki_Di_sum);
        for(uint i = 0; i < 6; i++){
            Q[indx - 1] += body1->Bi[i]*(body1->Li[i] - body1->Ki_Di_sum[i]);
        }
    }

#if 0
    memset(M, 0, sizeof(double)*num_body*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        for(uint indx2 = 1; indx2 <= num_body; indx2++){
            body2 = &body[indx2];
            double Ki_Bi[6] = {0,}, temp = 0;
            if (indx == indx2){
                mat(body1->Ki, body1->Bi, 6, 6, 6, Ki_Bi);
            }
            else if(indx < indx2){
                mat(body2->Ki, body2->Bi, 6, 6, 6, Ki_Bi);
            }
            else if(indx > indx2){
                mat(body1->Ki, body2->Bi, 6, 6, 6, Ki_Bi);
            }
            temp = 0;
            for(uint i = 0; i < 6; i++){
                temp += body1->Bi[i]*Ki_Bi[i];
            }
            M[(indx - 1)*num_body + (indx2 - 1)] = temp;
        }
    }

    int *indx = new int[num_body];
    double *fac = new double[num_body*num_body];
    double *q_ddot = new double[num_body];

    numeric->ludcmp(M, static_cast<int>(num_body), indx, 0.0, fac);
    numeric->lubksb(fac, static_cast<int>(num_body), indx, Q, q_ddot);

    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].qi_ddot = q_ddot[indx - 1];
    }

    delete[] indx;
    delete[] fac;
    delete[] q_ddot;
#endif
}

void RobotArm::save_data() {
    fprintf(fp, "%.7f\t", t_current);
    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi);
    }

    kinematics();

    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].re[0], body[num_body].re[1], body[num_body].re[2]);
    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].ori[0], body[num_body].ori[1], body[num_body].ori[2]);

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_dot);
    }

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_ddot);
    }
    fprintf(fp, "\n");
}
