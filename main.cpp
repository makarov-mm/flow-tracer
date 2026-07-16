// FlowTracer — 3D lattice Boltzmann (D3Q19, BGK + Smagorinsky LES)
// Channel flow past an obstacle (sphere / cylinder / cube). The flow is made
// visible with massless tracer particles advected through the velocity field,
// rendered as glowing points with accumulation trails. Orbit camera.
// C++20 / Qt6 Widgets, multithreaded CPU (persistent worker pool), no other
// dependencies.

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QStyleFactory>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <numbers>
#include <random>
#include <stop_token>
#include <thread>
#include <vector>

// ------------------------------------------------------------------ thread pool

// Persistent worker pool. The old code spawned and joined a fresh team of
// std::threads for every parallel section — about a dozen times per frame.
// Here the workers are created once (std::jthread, stopped via stop_token on
// destruction) and jobs are handed out as [begin, end) chunks through an
// atomic cursor, so a frame costs only condition-variable wakeups.
class ThreadPool
{
public:
    ThreadPool()
    {
        const unsigned n = std::max(2u, std::thread::hardware_concurrency());
        m_workers.reserve(n);
        for (unsigned t = 0; t < n; ++t)
            m_workers.emplace_back([this](std::stop_token st) { workerLoop(st); });
    }

    ~ThreadPool()
    {
        {
            const std::lock_guard lock(m_mutex);
            for (auto &w : m_workers)
                w.request_stop();
        }
        m_start.notify_all();
    }

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool &operator=(const ThreadPool &) = delete;

    // Runs fn(begin, end) over [0, count) on the pool; blocks until done.
    void parallelFor(int count, const std::function<void(int, int)> &fn)
    {
        if (count <= 0)
            return;
        const int T = int(m_workers.size());
        {
            const std::lock_guard lock(m_mutex);
            m_fn = &fn;
            m_count = count;
            m_chunk = std::max(1, count / (T * 4));
            m_next.store(0, std::memory_order_relaxed);
            m_working = T;
            ++m_generation;
        }
        m_start.notify_all();
        std::unique_lock lock(m_mutex);
        m_done.wait(lock, [this] { return m_working == 0; });
        m_fn = nullptr;
    }

private:
    void workerLoop(std::stop_token st)
    {
        std::uint64_t seen = 0;
        for (;;) {
            const std::function<void(int, int)> *fn = nullptr;
            {
                std::unique_lock lock(m_mutex);
                m_start.wait(lock, [&] {
                    return st.stop_requested() || m_generation != seen;
                });
                if (st.stop_requested())
                    return;
                seen = m_generation;
                fn = m_fn;
            }
            for (;;) {
                const int b = m_next.fetch_add(m_chunk, std::memory_order_relaxed);
                if (b >= m_count)
                    break;
                (*fn)(b, std::min(m_count, b + m_chunk));
            }
            {
                const std::lock_guard lock(m_mutex);
                if (--m_working == 0)
                    m_done.notify_all();
            }
        }
    }

    std::vector<std::jthread> m_workers;
    std::mutex m_mutex;
    std::condition_variable m_start, m_done;
    const std::function<void(int, int)> *m_fn = nullptr;
    std::atomic<int> m_next{0};
    int m_count = 0, m_chunk = 1, m_working = 0;
    std::uint64_t m_generation = 0;
};

// ---------------------------------------------------------------- D3Q19 lattice

inline constexpr int Q = 19;
inline constexpr std::array<int, Q> EX = { 0, 1,-1, 0, 0, 0, 0, 1,-1, 1,-1, 1,-1, 1,-1, 0, 0, 0, 0 };
inline constexpr std::array<int, Q> EY = { 0, 0, 0, 1,-1, 0, 0, 1,-1,-1, 1, 0, 0, 0, 0, 1,-1, 1,-1 };
inline constexpr std::array<int, Q> EZ = { 0, 0, 0, 0, 0, 1,-1, 0, 0, 0, 0, 1,-1,-1, 1, 1,-1,-1, 1 };
inline constexpr std::array<int, Q> OPP = { 0, 2, 1, 4, 3, 6, 5, 8, 7,10, 9,12,11,14,13,16,15,18,17 };
inline constexpr std::array<float, Q> W = {
    1.f/3.f,
    1.f/18.f, 1.f/18.f, 1.f/18.f, 1.f/18.f, 1.f/18.f, 1.f/18.f,
    1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f,
    1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f, 1.f/36.f
};

enum class Obstacle { Sphere, CylinderZ, Cube };
enum class ColorMode { Stripes, Speed, Vorticity };

struct SimParams {
    float omega        = 1.94f;
    float u0           = 0.08f;
    int   stepsPerFrame = 5;
    bool  smagorinsky  = true;
    float cs           = 0.12f;   // Smagorinsky constant
    Obstacle obstacle  = Obstacle::Sphere;
    float obstacleR    = 0.16f;   // fraction of NY
    int   tracers      = 150000;
    float trailDecay   = 0.90f;
    float brightness   = 1.0f;
    ColorMode colorMode = ColorMode::Stripes;
    bool  paused       = false;
};

class Lbm3dCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit Lbm3dCanvas(QWidget *parent = nullptr)
    {
        setMinimumSize(640, 480);
        setGridPreset(1);
        connect(&m_timer, &QTimer::timeout, this, &Lbm3dCanvas::frame);
        m_timer.start(16);
    }

    SimParams params;

    void setGridPreset(int idx)
    {
        static const int dims[3][3] = { {96, 48, 48}, {128, 64, 64}, {160, 80, 80} };
        m_nx = dims[idx][0]; m_ny = dims[idx][1]; m_nz = dims[idx][2];
        reset();
    }

    void reset()
    {
        const size_t total = size_t(m_nx) * m_ny * m_nz;
        for (int q = 0; q < Q; ++q) {
            m_f[q].assign(total, 0.f);
            m_fp[q].assign(total, 0.f);
        }
        m_u.assign(total, 0.f);
        m_v.assign(total, 0.f);
        m_w.assign(total, 0.f);
        m_solid.assign(total, 0);
        buildObstacle();

        const float u0 = params.u0;
        for (size_t i = 0; i < total; ++i) {
            const float ux = m_solid[i] ? 0.f : u0;
            for (int q = 0; q < Q; ++q)
                m_f[q][i] = feq(q, 1.f, ux, 0.f, 0.f);
            m_u[i] = ux;
        }
        seedTracers();
    }

    void buildObstacle()
    {
        std::ranges::fill(m_solid, std::uint8_t{0});
        // channel walls in y and z
        for (int z = 0; z < m_nz; ++z)
            for (int y = 0; y < m_ny; ++y)
                for (int x = 0; x < m_nx; ++x)
                    if (y == 0 || y == m_ny - 1 || z == 0 || z == m_nz - 1)
                        m_solid[idx(x, y, z)] = 1;

        const float cx = m_nx * 0.25f;
        const float cy = m_ny * 0.5f + 1.5f;   // off-center: break symmetry
        const float cz = m_nz * 0.5f + 1.0f;
        const float R = params.obstacleR * m_ny;
        m_obsX = cx; m_obsY = cy; m_obsZ = cz; m_obsR = R;

        for (int z = 1; z < m_nz - 1; ++z)
            for (int y = 1; y < m_ny - 1; ++y)
                for (int x = 0; x < m_nx; ++x) {
                    float dx = x - cx, dy = y - cy, dz = z - cz;
                    bool in = false;
                    switch (params.obstacle) {
                    case Obstacle::Sphere:
                        in = dx * dx + dy * dy + dz * dz <= R * R; break;
                    case Obstacle::CylinderZ:
                        in = dx * dx + dy * dy <= R * R; break;
                    case Obstacle::Cube:
                        in = std::fabs(dx) <= R && std::fabs(dy) <= R
                          && std::fabs(dz) <= R; break;
                    }
                    if (in) m_solid[idx(x, y, z)] = 1;
                }
    }

    void seedTracers()
    {
        m_tx.resize(params.tracers);
        m_ty.resize(params.tracers);
        m_tz.resize(params.tracers);
        m_thue.resize(params.tracers);
        std::mt19937 rng(0xB0DE5);
        for (int i = 0; i < params.tracers; ++i)
            respawn(i, rng, true);
    }

    float msPerStep() const { return m_msPerStep; }
    float reynolds() const
    {
        float nu = (1.f / params.omega - 0.5f) / 3.f;
        return params.u0 * (2.f * m_obsR) / std::max(nu, 1e-6f);
    }

signals:
    void statsChanged();

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.drawImage(0, 0, m_image);
    }

    void resizeEvent(QResizeEvent *) override
    {
        m_image = QImage(std::max(64, width()), std::max(64, height()),
                         QImage::Format_RGB32);
        m_accum.assign(size_t(m_image.width()) * m_image.height() * 3, 0.f);
    }

    void mousePressEvent(QMouseEvent *e) override { m_last = e->pos(); }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        if (e->buttons() & Qt::LeftButton) {
            m_yaw   += (e->pos().x() - m_last.x()) * 0.008f;
            m_pitch += (e->pos().y() - m_last.y()) * 0.008f;
            m_pitch = std::clamp(m_pitch, -1.5f, 1.5f);
            m_last = e->pos();
        }
    }
    void wheelEvent(QWheelEvent *e) override
    {
        m_zoom *= std::pow(1.0015f, float(e->angleDelta().y()));
        m_zoom = std::clamp(m_zoom, 0.3f, 4.f);
    }

private slots:
    void frame()
    {
        if (!params.paused) {
            QElapsedTimer t; t.start();
            for (int s = 0; s < params.stepsPerFrame; ++s)
                lbmStep();
            m_msPerStep = float(t.nsecsElapsed()) * 1e-6f / params.stepsPerFrame;
            advectTracers(float(params.stepsPerFrame));
        }
        render();
        update();
        if (qEnvironmentVariableIsSet("DUMP_FRAMES")) {
            static int left = qEnvironmentVariableIntValue("DUMP_FRAMES");
            if (--left <= 0) { m_image.save("dump.png"); QApplication::quit(); }
        }
        if (!m_statClock.isValid() || m_statClock.hasExpired(400)) {
            emit statsChanged();
            m_statClock.restart();
        }
    }

private:
    size_t idx(int x, int y, int z) const
    {
        return (size_t(z) * m_ny + y) * m_nx + x;
    }

    // parallel over (y,z) rows of length nx
    template <class F> void parallelRows(F &&fn)
    {
        m_pool.parallelFor(m_ny * m_nz, [this, &fn](int b, int e) {
            for (int r = b; r < e; ++r)
                fn(r % m_ny, r / m_ny); // (y, z)
        });
    }

    static constexpr float feq(int q, float rho, float ux, float uy, float uz)
    {
        float eu = EX[q] * ux + EY[q] * uy + EZ[q] * uz;
        float u2 = ux * ux + uy * uy + uz * uz;
        return W[q] * rho * (1.f + 3.f * eu + 4.5f * eu * eu - 1.5f * u2);
    }

    void lbmStep()
    {
        const float omega0 = params.omega;
        const float u0 = params.u0;
        const bool les = params.smagorinsky;
        const float cs2 = params.cs * params.cs;
        const int nx = m_nx, ny = m_ny, nz = m_nz;

        // ---- collide into m_fp
        parallelRows([&](int y, int z) {
            for (int x = 0; x < nx; ++x) {
                const size_t i = idx(x, y, z);
                if (m_solid[i]) {
                    for (int q = 0; q < Q; ++q) m_fp[q][i] = m_f[q][i];
                    m_u[i] = m_v[i] = m_w[i] = 0.f;
                    continue;
                }
                float fq[Q], rho = 0.f, ux = 0.f, uy = 0.f, uz = 0.f;
                for (int q = 0; q < Q; ++q) {
                    fq[q] = m_f[q][i];
                    rho += fq[q];
                    ux += fq[q] * EX[q];
                    uy += fq[q] * EY[q];
                    uz += fq[q] * EZ[q];
                }
                const float inv = 1.f / rho;
                ux *= inv; uy *= inv; uz *= inv;
                float sp = std::sqrt(ux * ux + uy * uy + uz * uz);
                if (sp > 0.3f) {
                    float k = 0.3f / sp;
                    ux *= k; uy *= k; uz *= k;
                }
                m_u[i] = ux; m_v[i] = uy; m_w[i] = uz;

                float e[Q];
                for (int q = 0; q < Q; ++q)
                    e[q] = feq(q, rho, ux, uy, uz);

                float omega = omega0;
                if (les) {
                    // Smagorinsky: local relaxation from non-equilibrium stress
                    float pxx = 0, pyy = 0, pzz = 0, pxy = 0, pxz = 0, pyz = 0;
                    for (int q = 0; q < Q; ++q) {
                        const float d = fq[q] - e[q];
                        pxx += d * EX[q] * EX[q];
                        pyy += d * EY[q] * EY[q];
                        pzz += d * EZ[q] * EZ[q];
                        pxy += d * EX[q] * EY[q];
                        pxz += d * EX[q] * EZ[q];
                        pyz += d * EY[q] * EZ[q];
                    }
                    const float pi = std::sqrt(pxx * pxx + pyy * pyy + pzz * pzz
                        + 2.f * (pxy * pxy + pxz * pxz + pyz * pyz));
                    const float tau0 = 1.f / omega0;
                    const float tau = 0.5f * (tau0 + std::sqrt(tau0 * tau0
                        + 18.f * std::numbers::sqrt2_v<float> * cs2 * pi / rho));
                    omega = 1.f / tau;
                }
                for (int q = 0; q < Q; ++q)
                    m_fp[q][i] = fq[q] + omega * (e[q] - fq[q]);
            }
        });

        // ---- stream (pull) with half-way bounce-back
        parallelRows([&](int y, int z) {
            for (int x = 0; x < nx; ++x) {
                const size_t i = idx(x, y, z);
                if (m_solid[i]) continue;
                for (int q = 0; q < Q; ++q) {
                    const int sx = x - EX[q];
                    const int sy = y - EY[q];
                    const int sz = z - EZ[q];
                    if (sx < 0 || sx >= nx) { m_f[q][i] = m_fp[q][i]; continue; }
                    const size_t s = idx(sx, sy, sz); // sy, sz stay in range: walls are solid
                    m_f[q][i] = m_solid[s] ? m_fp[OPP[q]][i] : m_fp[q][s];
                }
            }
        });

        // ---- inlet / outlet
        parallelRows([&](int y, int z) {
            if (y == 0 || y == ny - 1 || z == 0 || z == nz - 1) return;
            const size_t iIn = idx(0, y, z);
            for (int q = 0; q < Q; ++q)
                m_f[q][iIn] = feq(q, 1.f, u0, 0.f, 0.f);
            const size_t iOut = idx(nx - 1, y, z), iPrev = idx(nx - 2, y, z);
            for (int q = 0; q < Q; ++q)
                m_f[q][iOut] = m_f[q][iPrev];
        });
    }

    // ------------------------------------------------------------- tracers

    template <class RNG> void respawn(int i, RNG &rng, bool anywhere = false)
    {
        std::uniform_real_distribution<float> uy(1.5f, m_ny - 2.5f);
        std::uniform_real_distribution<float> uz(1.5f, m_nz - 2.5f);
        std::uniform_real_distribution<float> ux(1.f, anywhere ? m_nx - 2.f : 4.f);
        float x, y, z;
        do {
            x = ux(rng); y = uy(rng); z = uz(rng);
        } while (m_solid[idx(int(x), int(y), int(z))]);
        m_tx[i] = x; m_ty[i] = y; m_tz[i] = z;
        m_thue[i] = (z - 1.5f) / (m_nz - 4.f); // stripe hue by seed height
    }

    void sampleVel(float x, float y, float z, float &vx, float &vy, float &vz) const
    {
        const int x0 = int(x), y0 = int(y), z0 = int(z);
        const float fx = x - x0, fy = y - y0, fz = z - z0;
        vx = vy = vz = 0.f;
        for (int dz = 0; dz < 2; ++dz)
            for (int dy = 0; dy < 2; ++dy)
                for (int dx = 0; dx < 2; ++dx) {
                    const float w = (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy)
                                  * (dz ? fz : 1 - fz);
                    const size_t i = idx(std::min(x0 + dx, m_nx - 1),
                                         std::min(y0 + dy, m_ny - 1),
                                         std::min(z0 + dz, m_nz - 1));
                    vx += w * m_u[i]; vy += w * m_v[i]; vz += w * m_w[i];
                }
    }

    void advectTracers(float dt)
    {
        m_pool.parallelFor(int(m_tx.size()), [this, dt](int b, int e) {
            std::mt19937 rng(0x51ED5u + std::uint32_t(b) * 7919u + m_tick);
            for (int i = b; i < e; ++i) {
                float vx, vy, vz;
                sampleVel(m_tx[i], m_ty[i], m_tz[i], vx, vy, vz);
                m_tx[i] += vx * dt;
                m_ty[i] += vy * dt;
                m_tz[i] += vz * dt;
                const bool out = m_tx[i] < 1.f || m_tx[i] > m_nx - 2.f
                              || m_ty[i] < 1.f || m_ty[i] > m_ny - 2.f
                              || m_tz[i] < 1.f || m_tz[i] > m_nz - 2.f;
                if (out || m_solid[idx(int(m_tx[i]), int(m_ty[i]), int(m_tz[i]))])
                    respawn(i, rng);
            }
        });
        ++m_tick;
    }

    // -------------------------------------------------------------- render

    void hsv(float h, float s, float v, float &r, float &g, float &b) const
    {
        h = h - std::floor(h);
        const float h6 = h * 6.f;
        const int i = int(h6);
        const float f = h6 - i;
        const float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
        switch (i % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
        }
    }

    void project(float x, float y, float z, float &sx, float &sy, float &depth) const
    {
        // center, rotate (yaw around Y-screen axis, pitch around X)
        const float cx = x - m_nx * 0.5f;
        const float cy = y - m_ny * 0.5f;
        const float cz = z - m_nz * 0.5f;
        const float cyaw = std::cos(m_yaw), syaw = std::sin(m_yaw);
        const float cpit = std::cos(m_pitch), spit = std::sin(m_pitch);
        // world x along screen-x, world y up
        float rx = cx * cyaw + cz * syaw;
        float rz = -cx * syaw + cz * cyaw;
        float ry = cy * cpit - rz * spit;
        rz = cy * spit + rz * cpit;
        const float f = m_nx * 2.5f;
        const float persp = f / (f + rz);
        const float s = std::min(m_image.width(), m_image.height())
                      / (m_nx * 1.25f) * m_zoom;
        sx = m_image.width() * 0.5f + rx * s * persp;
        sy = m_image.height() * 0.5f - ry * s * persp;
        depth = persp;
    }

    void render()
    {
        const int W2 = m_image.width(), H2 = m_image.height();
        if (m_accum.size() != size_t(W2) * H2 * 3)
            m_accum.assign(size_t(W2) * H2 * 3, 0.f);

        // fade trails
        const float d = params.trailDecay;
        m_pool.parallelFor(int(m_accum.size()), [this, d](int b, int e) {
            for (int i = b; i < e; ++i)
                m_accum[i] *= d;
        });

        // deposit tracers (single-threaded: benign and fast)
        const int n = int(m_tx.size());
        const float invU = 1.f / std::max(params.u0 * 1.6f, 1e-4f);
        const float bright = params.brightness * 0.55f;
        for (int i = 0; i < n; ++i) {
            float sx, sy, depth;
            project(m_tx[i], m_ty[i], m_tz[i], sx, sy, depth);
            const int px = int(sx), py = int(sy);
            if (unsigned(px) >= unsigned(W2) || unsigned(py) >= unsigned(H2)) continue;

            float r, g, b;
            if (params.colorMode == ColorMode::Stripes) {
                hsv(0.5f + m_thue[i] * 0.75f, 0.75f, 1.f, r, g, b);
            } else {
                float vx, vy, vz;
                sampleVel(m_tx[i], m_ty[i], m_tz[i], vx, vy, vz);
                const float sp = std::sqrt(vx * vx + vy * vy + vz * vz) * invU;
                if (params.colorMode == ColorMode::Speed) {
                    r = std::clamp(sp * 2.f, 0.f, 1.f);
                    g = std::clamp(sp * 1.2f - 0.15f, 0.f, 1.f);
                    b = std::clamp(0.5f - sp * 0.3f + (sp > 0.8f ? sp - 0.8f : 0.f), 0.f, 1.f);
                } else {
                    // vorticity magnitude via neighboring cells
                    const int x0 = int(m_tx[i]), y0 = int(m_ty[i]), z0 = int(m_tz[i]);
                    auto U = [&](int a, int c, int e2) { return m_u[idx(a, c, e2)]; };
                    auto V = [&](int a, int c, int e2) { return m_v[idx(a, c, e2)]; };
                    auto Wc = [&](int a, int c, int e2) { return m_w[idx(a, c, e2)]; };
                    const int xm = std::max(1, x0 - 1), xp2 = std::min(m_nx - 2, x0 + 1);
                    const int ym = std::max(1, y0 - 1), yp2 = std::min(m_ny - 2, y0 + 1);
                    const int zm = std::max(1, z0 - 1), zp2 = std::min(m_nz - 2, z0 + 1);
                    const float wx = Wc(x0, yp2, z0) - Wc(x0, ym, z0) - (V(x0, y0, zp2) - V(x0, y0, zm));
                    const float wy = U(x0, y0, zp2) - U(x0, y0, zm) - (Wc(xp2, y0, z0) - Wc(xm, y0, z0));
                    const float wz = V(xp2, y0, z0) - V(xm, y0, z0) - (U(x0, yp2, z0) - U(x0, ym, z0));
                    const float vo = std::clamp(std::sqrt(wx * wx + wy * wy + wz * wz)
                                                * invU * 3.f, 0.f, 1.f);
                    hsv(0.55f - vo * 0.55f, 0.85f, 0.25f + 0.75f * vo, r, g, b);
                }
            }
            const float k = bright * std::clamp(depth, 0.55f, 1.6f);
            float *acc = &m_accum[3 * (size_t(py) * W2 + px)];
            acc[0] += r * k; acc[1] += g * k; acc[2] += b * k;
        }

        // tone map into image
        m_pool.parallelFor(H2, [this, W2](int yb, int ye) {
            for (int y = yb; y < ye; ++y) {
                QRgb *out = reinterpret_cast<QRgb *>(m_image.scanLine(y));
                const float *acc = &m_accum[3 * size_t(y) * W2];
                for (int x = 0; x < W2; ++x) {
                    const float r = 1.f - std::exp(-acc[3 * x]);
                    const float g = 1.f - std::exp(-acc[3 * x + 1]);
                    const float b = 1.f - std::exp(-acc[3 * x + 2]);
                    out[x] = qRgb(int(8 + r * 247), int(8 + g * 247),
                                  int(12 + b * 243));
                }
            }
        });

        // overlay: domain box + obstacle silhouette
        QPainter p(&m_image);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QColor(90, 100, 120, 150));
        auto line3 = [&](float x0, float y0, float z0, float x1, float y1, float z1) {
            float ax, ay, az, bx, by, bz;
            project(x0, y0, z0, ax, ay, az);
            project(x1, y1, z1, bx, by, bz);
            p.drawLine(QPointF(ax, ay), QPointF(bx, by));
        };
        const float X = m_nx - 1.f, Y = m_ny - 1.f, Z = m_nz - 1.f;
        line3(0,0,0, X,0,0); line3(0,Y,0, X,Y,0); line3(0,0,Z, X,0,Z); line3(0,Y,Z, X,Y,Z);
        line3(0,0,0, 0,Y,0); line3(X,0,0, X,Y,0); line3(0,0,Z, 0,Y,Z); line3(X,0,Z, X,Y,Z);
        line3(0,0,0, 0,0,Z); line3(X,0,0, X,0,Z); line3(0,Y,0, 0,Y,Z); line3(X,Y,0, X,Y,Z);

        p.setPen(QColor(150, 155, 170, 190));
        const int SEG = 36;
        for (int ring = 0; ring < 3; ++ring) {
            QPointF prev;
            for (int s = 0; s <= SEG; ++s) {
                const float a = 2.f * std::numbers::pi_v<float> * s / SEG;
                float px3 = m_obsX, py3 = m_obsY, pz3 = m_obsZ;
                const float R = m_obsR;
                if (ring == 0) { px3 += R * std::cos(a); py3 += R * std::sin(a); }
                if (ring == 1) { px3 += R * std::cos(a); pz3 += R * std::sin(a); }
                if (ring == 2) { py3 += R * std::cos(a); pz3 += R * std::sin(a); }
                float sx, sy, dz;
                project(px3, py3, pz3, sx, sy, dz);
                if (s) p.drawLine(prev, QPointF(sx, sy));
                prev = QPointF(sx, sy);
            }
        }
    }

    int m_nx = 128, m_ny = 64, m_nz = 64;
    std::vector<float> m_f[Q], m_fp[Q];
    std::vector<float> m_u, m_v, m_w;
    std::vector<std::uint8_t> m_solid;
    float m_obsX = 0, m_obsY = 0, m_obsZ = 0, m_obsR = 1;

    std::vector<float> m_tx, m_ty, m_tz, m_thue;
    std::uint32_t m_tick = 1;

    ThreadPool m_pool;

    QImage m_image{800, 600, QImage::Format_RGB32};
    std::vector<float> m_accum;
    float m_yaw = 0.55f, m_pitch = 0.32f, m_zoom = 1.f;
    QPoint m_last;

    QTimer m_timer;
    QElapsedTimer m_statClock;
    float m_msPerStep = 0.f;
};

// -------------------------------------------------------------------- window

class MainWindow : public QWidget
{
    Q_OBJECT
public:
    MainWindow()
    {
        setWindowTitle("FlowTracer — 3D flow visualization");
        auto *canvas = new Lbm3dCanvas;

        auto *grid = new QComboBox;
        grid->addItems({ "96 x 48 x 48", "128 x 64 x 64", "160 x 80 x 80" });
        grid->setCurrentIndex(1);

        auto *obstacle = new QComboBox;
        obstacle->addItems({ "Sphere", "Cylinder (z)", "Cube" });

        auto *obsR = new QDoubleSpinBox;
        obsR->setRange(0.06, 0.30); obsR->setDecimals(2); obsR->setSingleStep(0.02);
        obsR->setValue(canvas->params.obstacleR);

        auto *omega = new QDoubleSpinBox;
        omega->setRange(0.6, 1.99); omega->setDecimals(3); omega->setSingleStep(0.01);
        omega->setValue(canvas->params.omega);

        auto *u0 = new QDoubleSpinBox;
        u0->setRange(0.01, 0.12); u0->setDecimals(3); u0->setSingleStep(0.005);
        u0->setValue(canvas->params.u0);

        auto *steps = new QSpinBox;
        steps->setRange(1, 30);
        steps->setValue(canvas->params.stepsPerFrame);

        auto *les = new QCheckBox("Smagorinsky LES");
        les->setChecked(true);

        auto *csBox = new QDoubleSpinBox;
        csBox->setRange(0.05, 0.25); csBox->setDecimals(2); csBox->setSingleStep(0.01);
        csBox->setValue(canvas->params.cs);

        auto *tracers = new QSpinBox;
        tracers->setRange(20000, 500000); tracers->setSingleStep(10000);
        tracers->setValue(canvas->params.tracers);

        auto *decay = new QDoubleSpinBox;
        decay->setRange(0.5, 0.99); decay->setDecimals(2); decay->setSingleStep(0.01);
        decay->setValue(canvas->params.trailDecay);

        auto *bright = new QDoubleSpinBox;
        bright->setRange(0.2, 5.0); bright->setSingleStep(0.1);
        bright->setValue(canvas->params.brightness);

        auto *color = new QComboBox;
        color->addItems({ "Stripes", "Speed", "Vorticity" });

        auto *reset = new QPushButton("Reset flow");
        auto *pause = new QPushButton("Pause");
        pause->setCheckable(true);

        auto *hint = new QLabel("LMB drag: orbit\nWheel: zoom");
        hint->setStyleSheet("color:#889");
        auto *stats = new QLabel;
        stats->setStyleSheet("color:#8fa;font-family:monospace");

        auto *form = new QFormLayout;
        form->addRow("Grid", grid);
        form->addRow("Obstacle", obstacle);
        form->addRow("Obstacle R", obsR);
        form->addRow("Omega", omega);
        form->addRow("Inflow u0", u0);
        form->addRow("Steps/frame", steps);
        form->addRow(les);
        form->addRow("Smagorinsky Cs", csBox);
        form->addRow("Tracers", tracers);
        form->addRow("Trail decay", decay);
        form->addRow("Brightness", bright);
        form->addRow("Color", color);
        form->addRow(reset);
        form->addRow(pause);
        form->addRow(hint);
        form->addRow(stats);

        auto *group = new QGroupBox("Parameters");
        group->setLayout(form);
        group->setFixedWidth(280);

        auto *layout = new QHBoxLayout(this);
        layout->addWidget(group);
        layout->addWidget(canvas, 1);

        connect(grid, &QComboBox::currentIndexChanged, this, [canvas](int i) {
            canvas->setGridPreset(i);
        });
        connect(obstacle, &QComboBox::currentIndexChanged, this, [canvas](int i) {
            canvas->params.obstacle = Obstacle(i);
            canvas->reset();
        });
        connect(obsR, &QDoubleSpinBox::valueChanged, this, [canvas](double v) {
            canvas->params.obstacleR = float(v);
            canvas->reset();
        });
        connect(omega, &QDoubleSpinBox::valueChanged, this, [canvas](double v) { canvas->params.omega = float(v); });
        connect(u0, &QDoubleSpinBox::valueChanged, this, [canvas](double v) { canvas->params.u0 = float(v); });
        connect(steps, &QSpinBox::valueChanged, this, [canvas](int v) { canvas->params.stepsPerFrame = v; });
        connect(les, &QCheckBox::toggled, this, [canvas](bool b) { canvas->params.smagorinsky = b; });
        connect(csBox, &QDoubleSpinBox::valueChanged, this, [canvas](double v) { canvas->params.cs = float(v); });
        connect(tracers, &QSpinBox::valueChanged, this, [canvas](int v) {
            canvas->params.tracers = v;
            canvas->seedTracers();
        });
        connect(decay, &QDoubleSpinBox::valueChanged, this, [canvas](double v) { canvas->params.trailDecay = float(v); });
        connect(bright, &QDoubleSpinBox::valueChanged, this, [canvas](double v) { canvas->params.brightness = float(v); });
        connect(color, &QComboBox::currentIndexChanged, this, [canvas](int i) { canvas->params.colorMode = ColorMode(i); });
        connect(reset, &QPushButton::clicked, this, [canvas] { canvas->reset(); });
        connect(pause, &QPushButton::toggled, this, [canvas, pause](bool b) {
            canvas->params.paused = b;
            pause->setText(b ? "Resume" : "Pause");
        });
        connect(canvas, &Lbm3dCanvas::statsChanged, this, [canvas, stats] {
            stats->setText(QString("%1 ms/step\nRe ~ %2")
                           .arg(canvas->msPerStep(), 0, 'f', 1)
                           .arg(canvas->reynolds(), 0, 'f', 0));
        });

        resize(1280, 800);
    }
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    QPalette pal;
    pal.setColor(QPalette::Window, QColor(37, 37, 42));
    pal.setColor(QPalette::WindowText, QColor(220, 220, 224));
    pal.setColor(QPalette::Base, QColor(28, 28, 32));
    pal.setColor(QPalette::Text, QColor(220, 220, 224));
    pal.setColor(QPalette::Button, QColor(48, 48, 54));
    pal.setColor(QPalette::ButtonText, QColor(220, 220, 224));
    pal.setColor(QPalette::Highlight, QColor(70, 120, 200));
    app.setPalette(pal);

    MainWindow w;
    w.show();
    return app.exec();
}

#include "main.moc"
