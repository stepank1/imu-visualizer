#define DPS_TO_RADS 0.017453293f
#define RADS_TO_DGS 180 / M_PI

typedef struct vec3
{
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
} vec;

typedef struct quat
{
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
} quat;

quat update_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void normalize(quat *q);
quat gyro_to_quat(quat *last, float gx, float gy, float gz, float dt) ;
vec3 rotate(quat q, vec3 v);
vec3 rotate(vec3 v, quat q);
vec3 cross_product(vec3 v1, vec3 v2);
vec3 cross_product_earth(vec3 v1);
quat mult(quat *a, quat *b);
vec3 mult(float a, vec3 v);
quat sum(quat q, vec3 v);
vec3 sum(vec3 v1, vec3 v2);


