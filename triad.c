/*
 * ============================================================
 *              TRIAD ATTITUDE DETERMINATION
 *              STM32 / C ÖRNEK KODU
 * ============================================================
 *
 * Amaç:
 *   İki vektör kullanarak uydunun yönelimini (attitude)
 *   hesaplamak.
 *
 * Kullanılan vektörler:
 *
 *   1) Manyetometre -> Dünya manyetik alanı
 *   2) Güneş sensörü -> Güneş yönü
 *
 * TRIAD mantığı:
 *
 *   t1 = normalize(v1)
 *   t2 = normalize(t1 x v2)
 *   t3 = t1 x t2
 *
 * Body frame:
 *      Tb = [t1_body  t2_body  t3_body]
 *
 * Reference frame:
 *      Tr = [t1_ref   t2_ref   t3_ref]
 *
 * DCM:
 *
 *      Cbr = Tb * Tr^T
 *
 * Bu matris uydunun referans koordinat sistemine göre
 * yönelimini temsil eder.
 *
 * NOT:
 * Bu örnekte sensörlerden gerçek veri okunmuyor.
 * mag_body ve sun_body örnek olarak verilmiştir.
 * ============================================================
 */

#include <stdio.h>
#include <math.h>

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;


/* ------------------------------------------------------------
 * Vektör büyüklüğü
 * ------------------------------------------------------------ */
float vector_norm(Vector3 v)
{
    return sqrtf(v.x*v.x +
                 v.y*v.y +
                 v.z*v.z);
}


/* ------------------------------------------------------------
 * Vektör normalize
 *
 * v_hat = v / |v|
 * ------------------------------------------------------------ */
Vector3 normalize(Vector3 v)
{
    Vector3 result;

    float n = vector_norm(v);

    if(n < 0.000001f)
    {
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = 0.0f;
        return result;
    }

    result.x = v.x / n;
    result.y = v.y / n;
    result.z = v.z / n;

    return result;
}


/* ------------------------------------------------------------
 * Cross Product
 *
 * a x b =
 *
 * [ ay*bz - az*by ]
 * [ az*bx - ax*bz ]
 * [ ax*by - ay*bx ]
 * ------------------------------------------------------------ */
Vector3 cross(Vector3 a, Vector3 b)
{
    Vector3 r;

    r.x = a.y*b.z - a.z*b.y;
    r.y = a.z*b.x - a.x*b.z;
    r.z = a.x*b.y - a.y*b.x;

    return r;
}


/* ------------------------------------------------------------
 * TRIAD
 *
 * Body:
 *      v1_body = manyetometre
 *      v2_body = güneş sensörü
 *
 * Reference:
 *      v1_ref = manyetik alan referansı
 *      v2_ref = güneş referansı
 *
 * Sonuç:
 *      Cbr[3][3] = DCM
 * ------------------------------------------------------------ */
void TRIAD(Vector3 v1_body,
           Vector3 v2_body,
           Vector3 v1_ref,
           Vector3 v2_ref,
           float Cbr[3][3])
{
    Vector3 t1b, t2b, t3b;
    Vector3 t1r, t2r, t3r;

    /* ---------------- BODY FRAME ---------------- */

    /* İlk eksen */
    t1b = normalize(v1_body);

    /* İkinci eksen */
    t2b = cross(t1b, v2_body);
    t2b = normalize(t2b);

    /* Üçüncü eksen */
    t3b = cross(t1b, t2b);


    /* --------------- REFERENCE FRAME --------------- */

    /* İlk eksen */
    t1r = normalize(v1_ref);

    /* İkinci eksen */
    t2r = cross(t1r, v2_ref);
    t2r = normalize(t2r);

    /* Üçüncü eksen */
    t3r = cross(t1r, t2r);


    /*
     * Tb = [t1b t2b t3b]
     *
     * Tr = [t1r t2r t3r]
     *
     * Cbr = Tb * Tr^T
     *
     * Matris çarpımı:
     */

    Cbr[0][0] = t1b.x*t1r.x +
                t2b.x*t2r.x +
                t3b.x*t3r.x;

    Cbr[0][1] = t1b.x*t1r.y +
                t2b.x*t2r.y +
                t3b.x*t3r.y;

    Cbr[0][2] = t1b.x*t1r.z +
                t2b.x*t2r.z +
                t3b.x*t3r.z;


    Cbr[1][0] = t1b.y*t1r.x +
                t2b.y*t2r.x +
                t3b.y*t3r.x;

    Cbr[1][1] = t1b.y*t1r.y +
                t2b.y*t2r.y +
                t3b.y*t3r.y;

    Cbr[1][2] = t1b.y*t1r.z +
                t2b.y*t2r.z +
                t3b.y*t3r.z;


    Cbr[2][0] = t1b.z*t1r.x +
                t2b.z*t2r.x +
                t3b.z*t3r.x;

    Cbr[2][1] = t1b.z*t1r.y +
                t2b.z*t2r.y +
                t3b.z*t3r.y;

    Cbr[2][2] = t1b.z*t1r.z +
                t2b.z*t2r.z +
                t3b.z*t3r.z;
}


/* ------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------ */
int main(void)
{
    /*
     * BODY FRAME
     *
     * Gerçek uyduda bunlar sensörden gelecek:
     *
     * mag_body -> Magnetometer
     * sun_body -> Sun Sensor
     */

    Vector3 mag_body =
    {
        0.30f,
        0.40f,
        0.50f
    };

    Vector3 sun_body =
    {
        0.80f,
        0.20f,
        0.10f
    };


    /*
     * REFERENCE FRAME
     *
     * Gerçek uyduda:
     *
     * mag_ref -> Dünya manyetik modelinden
     * sun_ref -> Güneş konum modelinden
     */

    Vector3 mag_ref =
    {
        0.20f,
        0.50f,
        0.40f
    };

    Vector3 sun_ref =
    {
        0.90f,
        0.10f,
        0.20f
    };


    /* TRIAD sonucunda oluşacak DCM */
    float Cbr[3][3];


    /* TRIAD çalıştır */
    TRIAD(
        mag_body,
        sun_body,
        mag_ref,
        sun_ref,
        Cbr
    );


    /* Sonucu ekrana yazdır */
    printf("TRIAD DCM:\n\n");

    for(int i = 0; i < 3; i++)
    {
        printf(
            "%8.4f  %8.4f  %8.4f\n",
            Cbr[i][0],
            Cbr[i][1],
            Cbr[i][2]
        );
    }


    return 0;
}
