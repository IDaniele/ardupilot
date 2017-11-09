#include <AP_Math/AP_Math.h>
#include <cmath>

static double* transpose(double A[], int row, int col);
static double* multiply(double A[], double B[], int ra, int ca, int rb, int cb);
static double* multiplyscalar(double A[], double b, int ra, int ca);
static double* addmatrix(double A[], double B[], int row, int col);
static double* submatrix(double A[], double B[], int row, int col);
static double* invert(double A[], int dim);
static double frobeniusnorm(double A[], int row, int col);
static double topos(double a);

static bool inverse3x3(double m[],double invOut[]);
static bool inverse4x4(double m[],double invOut[]);
static bool mat_inverse(double* A, double* inv, uint8_t n);
static void mat_LU_decompose(double* A, double* L, double* U, double *P, uint8_t n);
static void mat_pivot(double* A, double* pivot, uint8_t n);
double* mat_mul(double *A, double *B, uint8_t n);
static void mat_forward_sub(double *L, double *out, uint8_t n);
static void mat_back_sub(double *U, double *out, uint8_t n);

static double topos(double a){if(a < 0.0){return -a;} return a;}

static void swap(double &a, double &b)
{
    double c;
    c = a;
    a = b;
    b = c;
}

static double* transpose(double A[], int row, int col){
    double *B = new double[col*row];
    for(int i=0; i<row; i++){
        for(int j=0; j<col; j++){
            B[(j*row)+i] = A[(i*col)+j];
        }
    }
    return B;
}

static double* multiply(double A[], double B[], int ra, int ca, int rb, int cb){
    if(ca != rb){
        //hal.console->printf("\n multiplication error\n");
        return nullptr;
    }
    double* C = new double[ra*cb];
    double sum = 0;
    for(int i = 0; i < ra; ++i){
        for(int j=0; j<cb; j++){
            for(int x=0; x<ca; x++){
                sum += A[(i*ca)+x]*B[(x*cb)+j];
            }
            C[(i*cb)+j] = sum;
            sum = 0;
        }
    }
    return C;
}

static double* multiplyscalar(double A[], double b, int ra, int ca){
    double *B = new double[ra*ca];
    for(int i=0; i<ra*ca; i++){
        B[i] = A[i] * b;
    }
    return B;
}

static double* addmatrix(double A[], double B[], int row, int col){
    double *C = new double[row*col];
    for(int i=0; i<row*col; i++){
            C[i] = A[i] + B[i];
    }
    return C;
}

static double* submatrix(double A[], double B[], int row, int col){
    double *C = new double[row*col];
    for(int i=0; i<row*col; i++){
            C[i] = A[i] - B[i];
    }
    return C;
}

static double frobeniusnorm(double A[], int row, int col){
    float norm = 0;
    for(int i=0; i<row; i++){
            for(int j=0; j<col; j++){
                norm += pow(A[(i*col)+j], 2);
            }
        }
    norm = sqrt(norm);
    return norm;
}

static double* invert(double A[], int dim){
    double *B;
    B = new double[dim*dim];
    switch(dim){
    case 3:
        inverse3x3(A,B);
        break;
    case 4:
        inverse4x4(A,B);
        break;
    default:
        mat_inverse(A, B, dim);
        break;
    }
    return B;
}

bool inverse3x3(double m[], double invOut[])
{
    double inv[9];
    // computes the inverse of a matrix m
    double  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if ((det == 0.0) || isinf(det)) {
        return false;
    }

    double invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint8_t i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

bool inverse4x4(double m[],double invOut[])
{
    double inv[16], det;
    uint8_t i;

    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0.0 || isinf(det)){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

static bool mat_inverse(double* A, double* inv, uint8_t n)
{
    double *L, *U, *P;
    bool ret = true;
    L = new double[n*n];
    U = new double[n*n];
    P = new double[n*n];
    mat_LU_decompose(A,L,U,P,n);

    double *L_inv = new double[n*n];
    double *U_inv = new double[n*n];

    memset(L_inv,0,n*n*sizeof(double));
    mat_forward_sub(L,L_inv,n);

    memset(U_inv,0,n*n*sizeof(double));
    mat_back_sub(U,U_inv,n);

    // decomposed matrices no longer required
    delete[] L;
    delete[] U;

    double *inv_unpivoted = mat_mul(U_inv,L_inv,n);
    double *inv_pivoted = mat_mul(inv_unpivoted, P, n);

    //check sanity of results
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(isnan(inv_pivoted[i*n+j]) || isinf(inv_pivoted[i*n+j])){
                ret = false;
            }
        }
    }
    memcpy(inv,inv_pivoted,n*n*sizeof(double));

    //free memory
    delete[] inv_pivoted;
    delete[] inv_unpivoted;
    delete[] P;
    delete[] U_inv;
    delete[] L_inv;
    return ret;
}

static void mat_LU_decompose(double* A, double* L, double* U, double *P, uint8_t n)
{
    memset(L,0,n*n*sizeof(double));
    memset(U,0,n*n*sizeof(double));
    memset(P,0,n*n*sizeof(double));
    mat_pivot(A,P,n);

    double *APrime = mat_mul(P,A,n);
    for(uint8_t i = 0; i < n; i++) {
        L[i*n + i] = 1;
    }
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(j <= i) {
                U[j*n + i] = APrime[j*n + i];
                for(uint8_t k = 0; k < j; k++) {
                    U[j*n + i] -= L[j*n + k] * U[k*n + i];
                }
            }
            if(j >= i) {
                L[j*n + i] = APrime[j*n + i];
                for(uint8_t k = 0; k < i; k++) {
                    L[j*n + i] -= L[j*n + k] * U[k*n + i];
                }
                L[j*n + i] /= U[i*n + i];
            }
        }
    }
    delete[] APrime;
}

static void mat_pivot(double* A, double* pivot, uint8_t n)
{
    for(uint8_t i = 0;i<n;i++){
        for(uint8_t j=0;j<n;j++) {
            pivot[i*n+j] = static_cast<double>(i==j);
        }
    }

    for(uint8_t i = 0;i < n; i++) {
        uint8_t max_j = i;
        for(uint8_t j=i;j<n;j++){
            if(fabsf(A[j*n + i]) > fabsf(A[max_j*n + i])) {
                max_j = j;
            }
        }

        if(max_j != i) {
            for(uint8_t k = 0; k < n; k++) {
                swap(pivot[i*n + k], pivot[max_j*n + k]);
            }
        }
    }
}

double* mat_mul(double *A, double *B, uint8_t n)
{
    double* ret = new double[n*n];
    memset(ret,0.0f,n*n*sizeof(double));

    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            for(uint8_t k = 0;k < n; k++) {
                ret[i*n + j] += A[i*n + k] * B[k*n + j];
            }
        }
    }
    return ret;
}

static void mat_forward_sub(double *L, double *out, uint8_t n)
{
    // Forward substitution solve LY = I
    for(int i = 0; i < n; i++) {
        out[i*n + i] = 1/L[i*n + i];
        for (int j = i+1; j < n; j++) {
            for (int k = i; k < j; k++) {
                out[j*n + i] -= L[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= L[j*n + j];
        }
    }
}

static void mat_back_sub(double *U, double *out, uint8_t n)
{
    // Backward Substitution solve UY = I
    for(int i = n-1; i >= 0; i--) {
        out[i*n + i] = 1/U[i*n + i];
        for (int j = i - 1; j >= 0; j--) {
            for (int k = i; k > j; k--) {
                out[j*n + i] -= U[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= U[j*n + j];
        }
    }
}
