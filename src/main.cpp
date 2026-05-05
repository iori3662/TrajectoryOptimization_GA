#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kEarthRadius = 6378137.0;
constexpr double kMu = 3.986004418e14;
constexpr double kRho0 = 1.225;
constexpr double kScaleHeight = 7200.0;

struct Vec3 {
    double x{}, y{}, z{};
    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
};

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

struct HimesVehicle {
    double mass = 32000.0;
    double refArea = 65.0;
    double maxThrust = 1.2e6;
    double isp = 320.0;
    std::array<double, 3> inertia = {2.1e6, 3.0e6, 2.7e6};
};

struct State {
    Vec3 r;     // inertial position [m]
    Vec3 v;     // inertial velocity [m/s]
    Vec3 eul;   // roll, pitch, yaw [rad]
    Vec3 w;     // body rates [rad/s]
    double mass;
};

struct Control {
    double thrustFrac;   // 0..1
    double alpha;        // angle of attack [rad]
    double bank;         // bank angle [rad]
    double yawCmd;       // yaw offset [rad]
};

struct Individual {
    std::vector<Control> genes;
    double fitness = std::numeric_limits<double>::infinity();
};

struct Target {
    double altitude;
    double speed;
    double gamma;
};

Vec3 gravityAccel(const Vec3& r) {
    const double rnorm = norm(r);
    return r * (-kMu / (rnorm * rnorm * rnorm));
}

double atmosphericDensity(double altitude) {
    if (altitude < 0.0) altitude = 0.0;
    return kRho0 * std::exp(-altitude / kScaleHeight);
}

Control clampControl(const Control& c) {
    Control out = c;
    out.thrustFrac = std::clamp(out.thrustFrac, 0.0, 1.0);
    out.alpha = std::clamp(out.alpha, -20.0 * kDeg2Rad, 35.0 * kDeg2Rad);
    out.bank = std::clamp(out.bank, -80.0 * kDeg2Rad, 80.0 * kDeg2Rad);
    out.yawCmd = std::clamp(out.yawCmd, -20.0 * kDeg2Rad, 20.0 * kDeg2Rad);
    return out;
}

State dynamics(const State& s, const Control& u, const HimesVehicle& veh) {
    const double rmag = norm(s.r);
    const double altitude = rmag - kEarthRadius;
    const double rho = atmosphericDensity(altitude);
    const double speed = std::max(1.0, norm(s.v));
    const double q = 0.5 * rho * speed * speed;

    // simplified HIMES coefficient model
    const double cl = 0.22 + 3.8 * u.alpha;
    const double cd = 0.025 + 0.12 * cl * cl;

    const double liftMag = q * veh.refArea * cl;
    const double dragMag = q * veh.refArea * cd;
    const double thrust = veh.maxThrust * u.thrustFrac;

    Vec3 vhat = s.v * (1.0 / speed);
    Vec3 rhat = s.r * (1.0 / rmag);
    Vec3 hhat = cross(s.r, s.v);
    const double hnorm = std::max(1e-6, norm(hhat));
    hhat = hhat * (1.0 / hnorm);
    Vec3 lhat = cross(hhat, vhat);

    Vec3 drag = vhat * (-dragMag / s.mass);
    Vec3 lift = lhat * (liftMag * std::cos(u.bank) / s.mass) + hhat * (liftMag * std::sin(u.bank) / s.mass);
    Vec3 thrustVec = (vhat * std::cos(u.alpha) + rhat * std::sin(u.alpha)) * (thrust / s.mass);

    State ds{};
    ds.r = s.v;
    ds.v = gravityAccel(s.r) + drag + lift + thrustVec;

    // simple attitude tracking dynamics
    const Vec3 cmdAngles = {u.bank, u.alpha, u.yawCmd};
    const Vec3 angleErr = {cmdAngles.x - s.eul.x, cmdAngles.y - s.eul.y, cmdAngles.z - s.eul.z};
    const Vec3 kP = {2.5, 2.5, 2.0};
    const Vec3 kD = {1.6, 1.6, 1.4};

    Vec3 torque{
        kP.x * angleErr.x - kD.x * s.w.x,
        kP.y * angleErr.y - kD.y * s.w.y,
        kP.z * angleErr.z - kD.z * s.w.z
    };

    ds.eul = s.w;
    ds.w = {
        torque.x / veh.inertia[0],
        torque.y / veh.inertia[1],
        torque.z / veh.inertia[2]
    };

    const double g0 = 9.80665;
    ds.mass = -thrust / (veh.isp * g0);
    return ds;
}

State rk4Step(const State& s, const Control& u, const HimesVehicle& veh, double dt) {
    auto addScaled = [](const State& a, const State& b, double c) {
        State out = a;
        out.r += b.r * c;
        out.v += b.v * c;
        out.eul += b.eul * c;
        out.w += b.w * c;
        out.mass += b.mass * c;
        return out;
    };

    State k1 = dynamics(s, u, veh);
    State k2 = dynamics(addScaled(s, k1, dt * 0.5), u, veh);
    State k3 = dynamics(addScaled(s, k2, dt * 0.5), u, veh);
    State k4 = dynamics(addScaled(s, k3, dt), u, veh);

    State n = s;
    n = addScaled(n, k1, dt / 6.0);
    n = addScaled(n, k2, dt / 3.0);
    n = addScaled(n, k3, dt / 3.0);
    n = addScaled(n, k4, dt / 6.0);
    n.mass = std::max(8000.0, n.mass);
    return n;
}

double flightPathAngle(const State& s) {
    double rmag = norm(s.r);
    double speed = std::max(1e-3, norm(s.v));
    double vr = dot(s.r, s.v) / rmag;
    return std::asin(std::clamp(vr / speed, -1.0, 1.0));
}

double evaluate(Individual& ind, const State& x0, const HimesVehicle& veh, const Target& tgt, double dt) {
    State s = x0;
    double penalty = 0.0;
    for (auto& g : ind.genes) {
        Control u = clampControl(g);
        s = rk4Step(s, u, veh, dt);
        if ((norm(s.r) - kEarthRadius) < -100.0) penalty += 1e8;
    }

    const double alt = norm(s.r) - kEarthRadius;
    const double vel = norm(s.v);
    const double gamma = flightPathAngle(s);

    penalty += std::pow((alt - tgt.altitude) / 1000.0, 2);
    penalty += std::pow((vel - tgt.speed) / 50.0, 2);
    penalty += std::pow((gamma - tgt.gamma) / (2.0 * kDeg2Rad), 2);
    penalty += 0.0003 * (x0.mass - s.mass); // fuel economy term

    ind.fitness = penalty;
    return ind.fitness;
}

}  // namespace

int main() {
    HimesVehicle himes;
    Target target{40000.0, 1500.0, 2.0 * kDeg2Rad};

    State x0{};
    x0.r = {kEarthRadius + 12000.0, 0.0, 0.0};
    x0.v = {750.0, 0.0, 0.0};
    x0.eul = {0.0, 4.0 * kDeg2Rad, 0.0};
    x0.w = {0.0, 0.0, 0.0};
    x0.mass = himes.mass;

    constexpr int kHorizon = 120;
    constexpr double kDt = 0.5;
    constexpr int kPop = 60;
    constexpr int kGen = 120;
    constexpr int kElite = 6;

    std::mt19937_64 rng(static_cast<uint64_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::normal_distribution<double> n01(0.0, 1.0);

    auto randomControl = [&]() {
        return Control{u01(rng), (-5.0 + 20.0 * u01(rng)) * kDeg2Rad, (-45.0 + 90.0 * u01(rng)) * kDeg2Rad,
                       (-8.0 + 16.0 * u01(rng)) * kDeg2Rad};
    };

    std::vector<Individual> pop(kPop);
    for (auto& i : pop) {
        i.genes.resize(kHorizon);
        for (auto& g : i.genes) g = randomControl();
        evaluate(i, x0, himes, target, kDt);
    }

    for (int gen = 0; gen < kGen; ++gen) {
        std::sort(pop.begin(), pop.end(), [](const auto& a, const auto& b) { return a.fitness < b.fitness; });

        std::vector<Individual> next;
        next.insert(next.end(), pop.begin(), pop.begin() + kElite);

        auto tournament = [&]() -> const Individual& {
            const int a = static_cast<int>(u01(rng) * kPop) % kPop;
            const int b = static_cast<int>(u01(rng) * kPop) % kPop;
            return (pop[a].fitness < pop[b].fitness) ? pop[a] : pop[b];
        };

        while (static_cast<int>(next.size()) < kPop) {
            const Individual& p1 = tournament();
            const Individual& p2 = tournament();
            Individual child;
            child.genes.resize(kHorizon);

            for (int k = 0; k < kHorizon; ++k) {
                child.genes[k] = (u01(rng) < 0.5) ? p1.genes[k] : p2.genes[k];
                if (u01(rng) < 0.15) {
                    child.genes[k].thrustFrac = std::clamp(child.genes[k].thrustFrac + 0.12 * n01(rng), 0.0, 1.0);
                    child.genes[k].alpha += 2.5 * kDeg2Rad * n01(rng);
                    child.genes[k].bank += 4.0 * kDeg2Rad * n01(rng);
                    child.genes[k].yawCmd += 2.0 * kDeg2Rad * n01(rng);
                }
            }
            evaluate(child, x0, himes, target, kDt);
            next.push_back(std::move(child));
        }

        pop = std::move(next);
        if (gen % 20 == 0 || gen == kGen - 1) {
            std::cout << "Generation " << std::setw(3) << gen
                      << " | best fitness: " << std::fixed << std::setprecision(4) << pop.front().fitness << '\n';
        }
    }

    std::sort(pop.begin(), pop.end(), [](const auto& a, const auto& b) { return a.fitness < b.fitness; });
    const auto& best = pop.front();
    std::cout << "\nBest trajectory chromosome (first 10 control knots):\n";
    for (int i = 0; i < 10; ++i) {
        const auto& c = best.genes[i];
        std::cout << i << ": T=" << c.thrustFrac
                  << ", alpha[deg]=" << c.alpha / kDeg2Rad
                  << ", bank[deg]=" << c.bank / kDeg2Rad
                  << ", yaw[deg]=" << c.yawCmd / kDeg2Rad << '\n';
    }

    return 0;
}

