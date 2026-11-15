/* BER: THE HOUSE - PS2 (fatia 1)
 * Uma sala, camera em primeira pessoa, lanterna, colisao simples.
 * Renderizador 3D proprio na CPU (EE) + gsKit para desenhar triangulos.
 *
 * Controles (DualShock 2):
 *   Analogico esquerdo = andar     Analogico direito = olhar
 *   R2 = correr                    X = ligar/desligar lanterna
 */
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sifrpc.h>
#include <loadfile.h>
#include <iopcontrol.h>
#include <gsKit.h>
#include <dmaKit.h>
#include <libpad.h>

typedef struct { float x, y, z; } V3;
static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 add(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 sub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 mul(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float len(V3 a) { return sqrtf(dot(a, a)); }
static inline V3 norm(V3 a) { float l = len(a); return l > 1e-6f ? mul(a, 1.0f / l) : a; }
static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

/* ---------- geometria ---------- */
typedef struct { V3 p[3]; V3 n; float r, g, b; } Tri;
#define MAXTRI 3000
static Tri tris[MAXTRI];
static int ntri = 0;

static unsigned hash32(unsigned x) { x ^= x >> 13; x *= 0x5bd1e995u; x ^= x >> 15; return x; }

static void addGrid(V3 o, V3 u, V3 v, int nu, int nv, float r, float g, float b, unsigned seed)
{
    V3 n = norm(cross(u, v));
    V3 du = mul(u, 1.0f / nu), dv = mul(v, 1.0f / nv);
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            if (ntri + 2 > MAXTRI) return;
            V3 a = add(o, add(mul(du, (float)i), mul(dv, (float)j)));
            V3 bb = add(a, du), c = add(a, dv), d = add(bb, dv);
            float k = 1.0f + (((i + j) & 1) ? 0.06f : -0.04f)
                      + ((hash32(i * 7 + j * 13 + seed) % 100) / 100.0f) * 0.08f - 0.04f;
            Tri t1 = {{a, bb, c}, n, r * k, g * k, b * k};
            Tri t2 = {{bb, d, c}, n, r * k, g * k, b * k};
            tris[ntri++] = t1;
            tris[ntri++] = t2;
        }
}

static void addBox(float cx, float cz, float w, float d, float h, float r, float g, float b, unsigned seed)
{
    float x0 = cx - w / 2, x1 = cx + w / 2, z0 = cz - d / 2, z1 = cz + d / 2;
    addGrid(v3(x0, h, z0), v3(w, 0, 0), v3(0, 0, d), 2, 2, r, g, b, seed);      /* topo   */
    addGrid(v3(x0, 0, z0), v3(w, 0, 0), v3(0, h, 0), 2, 2, r, g, b, seed + 1);  /* norte  */
    addGrid(v3(x0, 0, z1), v3(w, 0, 0), v3(0, h, 0), 2, 2, r, g, b, seed + 2);  /* sul    */
    addGrid(v3(x0, 0, z0), v3(0, 0, d), v3(0, h, 0), 2, 2, r, g, b, seed + 3);  /* oeste  */
    addGrid(v3(x1, 0, z0), v3(0, 0, d), v3(0, h, 0), 2, 2, r, g, b, seed + 4);  /* leste  */
}

/* caixas de colisao (minx,minz,maxx,maxz) */
typedef struct { float x0, z0, x1, z1; } Box;
static Box boxes[8];
static int nbox = 0;
static void colBox(float cx, float cz, float w, float d)
{
    Box b = {cx - w / 2, cz - d / 2, cx + w / 2, cz + d / 2};
    boxes[nbox++] = b;
}

static void buildWorld(void)
{
    /* sala 8 x 8 x 3 */
    addGrid(v3(-4, 0, -4), v3(8, 0, 0), v3(0, 0, 8), 10, 10, 0.30f, 0.24f, 0.18f, 10);  /* piso    */
    addGrid(v3(-4, 3, -4), v3(8, 0, 0), v3(0, 0, 8), 10, 10, 0.26f, 0.26f, 0.25f, 20);  /* teto    */
    addGrid(v3(-4, 0, -4), v3(8, 0, 0), v3(0, 3, 0), 10, 4, 0.42f, 0.40f, 0.34f, 30);   /* norte   */
    addGrid(v3(-4, 0, 4),  v3(8, 0, 0), v3(0, 3, 0), 10, 4, 0.42f, 0.40f, 0.34f, 40);   /* sul     */
    addGrid(v3(-4, 0, -4), v3(0, 0, 8), v3(0, 3, 0), 10, 4, 0.40f, 0.38f, 0.32f, 50);   /* oeste   */
    addGrid(v3(4, 0, -4),  v3(0, 0, 8), v3(0, 3, 0), 10, 4, 0.40f, 0.38f, 0.32f, 60);   /* leste   */
    /* moveis */
    addBox(1.5f, -1.0f, 1.6f, 0.9f, 0.8f, 0.35f, 0.22f, 0.12f, 100); colBox(1.5f, -1.0f, 1.6f, 0.9f);   /* mesa    */
    addBox(-3.4f, -2.5f, 1.2f, 0.7f, 2.2f, 0.20f, 0.14f, 0.09f, 200); colBox(-3.4f, -2.5f, 1.2f, 0.7f); /* armario */
    addBox(2.0f, 2.5f, 0.5f, 0.5f, 3.0f, 0.45f, 0.43f, 0.40f, 300); colBox(2.0f, 2.5f, 0.5f, 0.5f);    /* pilar   */
    addBox(0.0f, -3.9f, 1.1f, 0.12f, 2.2f, 0.03f, 0.02f, 0.02f, 400);                                   /* porta   */
    printf("triangulos: %d\n", ntri);
}

/* ---------- renderizacao ---------- */
typedef struct { float x, y, z, r, g, b; } CV;
typedef struct { float x[3], y[3], z; unsigned char r[3], g[3], b[3]; } Out;
#define MAXOUT 5000
static Out outs[MAXOUT];
static int nout;

static int cmpOut(const void *a, const void *b)
{
    float za = ((const Out *)a)->z, zb = ((const Out *)b)->z;
    return (za < zb) - (za > zb); /* mais longe primeiro */
}

static CV lerpCV(CV a, CV b, float t)
{
    CV r = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
            a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
    return r;
}

static void emit(GSGLOBAL *gs, CV a, CV b, CV c, float focal)
{
    if (nout >= MAXOUT) return;
    float cx = gs->Width * 0.5f, cy = gs->Height * 0.5f;
    CV v[3] = {a, b, c};
    Out o;
    int offL = 0, offR = 0, offT = 0, offB = 0;
    for (int i = 0; i < 3; i++) {
        float sx = cx + focal * v[i].x / v[i].z, sy = cy - focal * v[i].y / v[i].z;
        sx = clampf(sx, -1500.0f, gs->Width + 1500.0f);   /* guard band do GS */
        sy = clampf(sy, -1500.0f, gs->Height + 1500.0f);
        o.x[i] = sx; o.y[i] = sy;
        o.r[i] = (unsigned char)(clampf(v[i].r, 0, 1) * 255);
        o.g[i] = (unsigned char)(clampf(v[i].g, 0, 1) * 255);
        o.b[i] = (unsigned char)(clampf(v[i].b, 0, 1) * 255);
        offL += sx < 0; offR += sx > gs->Width; offT += sy < 0; offB += sy > gs->Height;
    }
    if (offL == 3 || offR == 3 || offT == 3 || offB == 3) return;
    o.z = (a.z + b.z + c.z) / 3.0f;
    outs[nout++] = o;
}

static void render(GSGLOBAL *gs, V3 cam, float yaw, float pitch, float flash, float t)
{
    V3 f = v3(sinf(yaw) * cosf(pitch), sinf(pitch), -cosf(yaw) * cosf(pitch));
    V3 r = v3(cosf(yaw), 0, sinf(yaw));
    V3 u = cross(r, f);
    float focal = (gs->Height * 0.5f) / tanf(36.0f * 3.14159265f / 180.0f);
    float flick = 0.94f + 0.06f * sinf(t * 31.0f) * sinf(t * 7.3f);
    const float cosOut = 0.80f, cosIn = 0.93f;
    nout = 0;

    for (int i = 0; i < ntri; i++) {
        const Tri *T = &tris[i];
        CV v[3];
        for (int k = 0; k < 3; k++) {
            V3 d = sub(T->p[k], cam);
            float dist = len(d) + 1e-4f;
            V3 dir = mul(d, 1.0f / dist);
            float ca = dot(dir, f);
            float s = clampf((ca - cosOut) / (cosIn - cosOut), 0, 1);
            s = s * s * (3 - 2 * s);
            float atten = 1.0f / (1.0f + 0.25f * dist + 0.06f * dist * dist);
            float ndl = fabsf(dot(T->n, dir));
            float I = 2.4f * flash * flick * s * atten * (0.35f + 0.65f * ndl);
            const float amb = 0.05f;
            v[k].x = dot(d, r); v[k].y = dot(d, u); v[k].z = dot(d, f);
            v[k].r = T->r * (amb * 0.6f + I);
            v[k].g = T->g * (amb * 0.8f + I * 0.93f);
            v[k].b = T->b * (amb * 1.2f + I * 0.78f);
        }
        /* recorte no plano proximo */
        const float nearZ = 0.25f;
        CV poly[4]; int np = 0;
        for (int k = 0; k < 3; k++) {
            CV a = v[k], b = v[(k + 1) % 3];
            int ina = a.z >= nearZ, inb = b.z >= nearZ;
            if (ina) poly[np++] = a;
            if (ina != inb) poly[np++] = lerpCV(a, b, (nearZ - a.z) / (b.z - a.z));
        }
        if (np < 3) continue;
        emit(gs, poly[0], poly[1], poly[2], focal);
        if (np == 4) emit(gs, poly[0], poly[2], poly[3], focal);
    }
    qsort(outs, nout, sizeof(Out), cmpOut);
    for (int i = 0; i < nout; i++) {
        const Out *o = &outs[i];
        gsKit_prim_triangle_gouraud(gs, o->x[0], o->y[0], 1, o->x[1], o->y[1], 1, o->x[2], o->y[2], 1,
            GS_SETREG_RGBAQ(o->r[0], o->g[0], o->b[0], 0x80, 0),
            GS_SETREG_RGBAQ(o->r[1], o->g[1], o->b[1], 0x80, 0),
            GS_SETREG_RGBAQ(o->r[2], o->g[2], o->b[2], 0x80, 0));
    }
}

/* ---------- controle ---------- */
static char padBuf[256] __attribute__((aligned(64)));

static void waitPad(void)
{
    int ret, n = 0;
    do { ret = padGetState(0, 0); n++; }
    while (ret != PAD_STATE_STABLE && ret != PAD_STATE_FINDCTP1 && n < 3000000);
}

static void padSetup(void)
{
    SifLoadFileInit();
    SifLoadModule("rom0:SIO2MAN", 0, NULL);
    SifLoadModule("rom0:PADMAN", 0, NULL);
    padInit(0);
    padPortOpen(0, 0, padBuf);
    waitPad();
    padSetMainMode(0, 0, PAD_MMODE_DUALSHOCK, PAD_MMODE_LOCK);
    waitPad();
}

static float axis(unsigned char v)
{
    float a = ((int)v - 128) / 128.0f;
    if (fabsf(a) < 0.2f) return 0;
    return a;
}

static int blocked(float x, float z)
{
    const float R = 0.28f;
    if (x < -4 + R || x > 4 - R || z < -4 + R || z > 4 - R) return 1;
    for (int i = 0; i < nbox; i++)
        if (x > boxes[i].x0 - R && x < boxes[i].x1 + R && z > boxes[i].z0 - R && z < boxes[i].z1 + R) return 1;
    return 0;
}

int main(void)
{
    SifInitRpc(0);
    SifIopReset("", 0);
    while (!SifIopSync()) {}
    SifInitRpc(0);

    GSGLOBAL *gs = gsKit_init_global();
    gs->ZBuffering = GS_SETTING_OFF;           /* usamos ordenacao de painter */
    gs->PrimAlphaEnable = GS_SETTING_OFF;
    dmaKit_init(D_CTRL_RELE_OFF, D_CTRL_MFD_OFF, D_CTRL_STS_UNSPEC, D_CTRL_STD_OFF, D_CTRL_RCYC_8, 1 << DMA_CHANNEL_GIF);
    dmaKit_chan_init(DMA_CHANNEL_GIF);
    gsKit_init_screen(gs);
    gsKit_mode_switch(gs, GS_ONESHOT);

    padSetup();
    buildWorld();

    V3 pos = v3(0, 1.65f, 3.0f);
    float yaw = 0, pitch = 0, bob = 0, bat = 1.0f, t = 0;
    int flashOn = 1;
    u32 oldPad = 0;
    float dt = (gs->Mode == GS_MODE_PAL) ? 0.02f : 0.0167f;
    struct padButtonStatus pb;

    while (1) {
        float lx = 0, ly = 0, rx = 0, ry = 0;
        u32 down = 0, held = 0;
        int st = padGetState(0, 0);
        if ((st == PAD_STATE_STABLE || st == PAD_STATE_FINDCTP1) && padRead(0, 0, &pb) != 0) {
            held = 0xffff ^ pb.btns;
            down = held & ~oldPad;
            oldPad = held;
            lx = axis(pb.ljoy_h); ly = axis(pb.ljoy_v);
            rx = axis(pb.rjoy_h); ry = axis(pb.rjoy_v);
        }
        if (down & PAD_CROSS) flashOn = !flashOn;

        yaw += rx * 2.4f * dt;
        pitch = clampf(pitch - ry * 1.6f * dt, -1.2f, 1.2f);

        float speed = (held & PAD_R2) ? 4.0f : 2.2f;
        V3 ff = v3(sinf(yaw), 0, -cosf(yaw)), fr = v3(cosf(yaw), 0, sinf(yaw));
        float mx = (fr.x * lx - ff.x * ly) * speed * dt;
        float mz = (fr.z * lx - ff.z * ly) * speed * dt;
        if (!blocked(pos.x + mx, pos.z)) pos.x += mx;
        if (!blocked(pos.x, pos.z + mz)) pos.z += mz;
        float moving = sqrtf(lx * lx + ly * ly);
        bob += moving * speed * dt * 3.2f;
        pos.y = 1.65f + sinf(bob) * 0.03f * moving;

        t += dt;
        if (flashOn) bat = clampf(bat - 0.004f * dt * 60.0f * dt, 0, 1);
        else bat = clampf(bat + 0.02f * dt, 0, 1);
        float flash = flashOn ? clampf(bat * 4.0f, 0, 1) : 0;
        if (flashOn && bat < 0.2f && (hash32((unsigned)(t * 20)) % 5) == 0) flash *= 0.2f;

        gsKit_clear(gs, GS_SETREG_RGBAQ(2, 3, 6, 0x80, 0));
        render(gs, pos, yaw, pitch, flash, t);

        /* HUD: mira e bateria */
        float cx = gs->Width * 0.5f, cy = gs->Height * 0.5f;
        gsKit_prim_sprite(gs, cx - 2, cy - 2, cx + 2, cy + 2, 2, GS_SETREG_RGBAQ(220, 230, 255, 0x80, 0));
        gsKit_prim_sprite(gs, 30, gs->Height - 40, 130, gs->Height - 30, 2, GS_SETREG_RGBAQ(20, 28, 50, 0x80, 0));
        gsKit_prim_sprite(gs, 32, gs->Height - 38, 32 + 96 * bat, gs->Height - 32, 3,
                          bat > 0.2f ? GS_SETREG_RGBAQ(230, 200, 100, 0x80, 0) : GS_SETREG_RGBAQ(220, 60, 50, 0x80, 0));

        gsKit_queue_exec(gs);
        gsKit_sync_flip(gs);
    }
    return 0;
}
