#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

// 角度変換係数
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

// 地球・大気モデル
constexpr double kEarthRadius = 6378137.0;
constexpr double kMu = 3.986004418e14;
constexpr double kRho0 = 1.225;
constexpr double kScaleHeight = 7200.0;
constexpr double kSpeedOfSound = 295.0;

struct Vec3 {
    double x{}, y{}, z{};
    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
};

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }
Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }

// クォータニオン（w, x, y, z）
struct Quaternion {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

Quaternion normalizeQ(const Quaternion& q) {
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Quaternion qMul(const Quaternion& a, const Quaternion& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

struct AeroPoint { double mach, alphaDeg, betaDeg, cl, cd, cy; };

// JAXA公開文献（HIMICO/HIMES, Mach5近傍の風洞比較図）を参考にした代表空力テーブル（近似）
const std::vector<double> kMachGrid{3.0, 5.0, 7.0};
const std::vector<double> kAlphaGrid{-15.0, 0.0, 15.0};
const std::vector<double> kBetaGrid{-6.0, 0.0, 6.0};

struct Coeff { double cl, cd, cy; };

Coeff coeffModel(double mach, double alphaDeg, double betaDeg) {
    // JAXA報告に示されたMach5・AoA範囲の傾向を保つ近似モデル（実機ではCSVテーブル置換を推奨）
    const double cl = 0.12 + 0.07 * (alphaDeg / 10.0) * (1.0 - 0.03*(mach-5.0));
    const double cd = 0.028 + 0.015 * std::pow(alphaDeg / 10.0, 2) + 0.002 * std::abs(betaDeg) / 10.0 + 0.0015*(mach-2.0);
    const double cy = 0.02 * (betaDeg / 10.0);
    return {cl, cd, cy};
}

double lerp(double a, double b, double t) { return a + (b - a) * t; }

// Mach-alpha-beta 3次元線形補間
Coeff interpolateAero(double mach, double alphaDeg, double betaDeg) {
    auto idx = [](const std::vector<double>& g, double v){
        double vv = std::clamp(v, g.front(), g.back());
        for (size_t i=0; i+1<g.size(); ++i) {
            if (vv >= g[i] && vv <= g[i+1]) return std::pair<size_t,double>{i, (vv-g[i])/(g[i+1]-g[i])};
        }
        return std::pair<size_t,double>{g.size()-2, 1.0};
    };
    auto [im, tm] = idx(kMachGrid, mach);
    auto [ia, ta] = idx(kAlphaGrid, alphaDeg);
    auto [ib, tb] = idx(kBetaGrid, betaDeg);

    Coeff c000 = coeffModel(kMachGrid[im],   kAlphaGrid[ia],   kBetaGrid[ib]);
    Coeff c100 = coeffModel(kMachGrid[im+1], kAlphaGrid[ia],   kBetaGrid[ib]);
    Coeff c010 = coeffModel(kMachGrid[im],   kAlphaGrid[ia+1], kBetaGrid[ib]);
    Coeff c110 = coeffModel(kMachGrid[im+1], kAlphaGrid[ia+1], kBetaGrid[ib]);
    Coeff c001 = coeffModel(kMachGrid[im],   kAlphaGrid[ia],   kBetaGrid[ib+1]);
    Coeff c101 = coeffModel(kMachGrid[im+1], kAlphaGrid[ia],   kBetaGrid[ib+1]);
    Coeff c011 = coeffModel(kMachGrid[im],   kAlphaGrid[ia+1], kBetaGrid[ib+1]);
    Coeff c111 = coeffModel(kMachGrid[im+1], kAlphaGrid[ia+1], kBetaGrid[ib+1]);

    auto tri = [&](double c000_,double c100_,double c010_,double c110_,double c001_,double c101_,double c011_,double c111_){
        double c00=lerp(c000_,c100_,tm), c10=lerp(c010_,c110_,tm), c01=lerp(c001_,c101_,tm), c11=lerp(c011_,c111_,tm);
        double c0=lerp(c00,c10,ta), c1=lerp(c01,c11,ta);
        return lerp(c0,c1,tb);
    };
    return {
        tri(c000.cl,c100.cl,c010.cl,c110.cl,c001.cl,c101.cl,c011.cl,c111.cl),
        tri(c000.cd,c100.cd,c010.cd,c110.cd,c001.cd,c101.cd,c011.cd,c111.cd),
        tri(c000.cy,c100.cy,c010.cy,c110.cy,c001.cy,c101.cy,c011.cy,c111.cy)
    };
}

struct HimesVehicle { double mass=32000.0, refArea=65.0, maxThrust=1.2e6, isp=320.0; std::array<double,3> I{2.1e6,3.0e6,2.7e6}; };
struct State { Vec3 r, v, w; Quaternion q; double mass; };
struct Control { double thrustFrac, alpha, beta, bank, yawCmd; };
struct Individual { std::vector<Control> genes; double fitness=1e30; };
struct Target { double altitude, speed, gamma; };

Vec3 gravityAccel(const Vec3& r){ double rn=norm(r); return r*(-kMu/(rn*rn*rn)); }
double rho(double h){ return kRho0*std::exp(-std::max(0.0,h)/kScaleHeight); }

Control clampControl(Control c){
    c.thrustFrac=std::clamp(c.thrustFrac,0.0,1.0);
    c.alpha=std::clamp(c.alpha,-20*kDeg2Rad,35*kDeg2Rad);
    c.beta=std::clamp(c.beta,-12*kDeg2Rad,12*kDeg2Rad);
    c.bank=std::clamp(c.bank,-80*kDeg2Rad,80*kDeg2Rad);
    c.yawCmd=std::clamp(c.yawCmd,-20*kDeg2Rad,20*kDeg2Rad);
    return c;
}

State dynamics(const State& s, const Control& uc, const HimesVehicle& hv){
    Control u=clampControl(uc);
    const double h=norm(s.r)-kEarthRadius, speed=std::max(1.0,norm(s.v));
    const double dens=rho(h), qdyn=0.5*dens*speed*speed;
    const double mach=speed/kSpeedOfSound;
    Coeff a=interpolateAero(mach,u.alpha*kRad2Deg,u.beta*kRad2Deg);

    Vec3 vhat=s.v*(1.0/speed), rhat=s.r*(1.0/norm(s.r));
    Vec3 hhat=cross(s.r,s.v); hhat=hhat*(1.0/std::max(1e-6,norm(hhat)));
    Vec3 lhat=cross(hhat,vhat);

    double L=qdyn*hv.refArea*a.cl, D=qdyn*hv.refArea*a.cd, Y=qdyn*hv.refArea*a.cy;
    double T=hv.maxThrust*u.thrustFrac;

    Vec3 fDrag=vhat*(-D/s.mass);
    Vec3 fLift=lhat*(L*std::cos(u.bank)/s.mass)+hhat*(L*std::sin(u.bank)/s.mass);
    Vec3 fSide=hhat*(Y/s.mass);
    Vec3 fThr=(vhat*std::cos(u.alpha)+rhat*std::sin(u.alpha))*(T/s.mass);

    State ds{};
    ds.r=s.v;
    ds.v=gravityAccel(s.r)+fDrag+fLift+fSide+fThr;

    // クォータニオン運動学: qdot = 0.5 * q * [0,w]
    Quaternion wq{0,s.w.x,s.w.y,s.w.z};
    Quaternion qdot=qMul(s.q,wq);
    ds.q={0.5*qdot.w,0.5*qdot.x,0.5*qdot.y,0.5*qdot.z};

    Vec3 cmdW{u.bank*0.15,u.alpha*0.12,u.yawCmd*0.12};
    ds.w={(cmdW.x-s.w.x)*0.8,(cmdW.y-s.w.y)*0.8,(cmdW.z-s.w.z)*0.8};
    ds.mass=-T/(hv.isp*9.80665);
    return ds;
}

State rk4(const State& s, const Control& u, const HimesVehicle& hv, double dt){
    auto add=[&](const State&a,const State&b,double c){ State o=a; o.r+=b.r*c; o.v+=b.v*c; o.w+=b.w*c; o.q={a.q.w+b.q.w*c,a.q.x+b.q.x*c,a.q.y+b.q.y*c,a.q.z+b.q.z*c}; o.mass+=b.mass*c; return o;};
    State k1=dynamics(s,u,hv), k2=dynamics(add(s,k1,dt*0.5),u,hv), k3=dynamics(add(s,k2,dt*0.5),u,hv), k4=dynamics(add(s,k3,dt),u,hv);
    State n=add(add(add(add(s,k1,dt/6),k2,dt/3),k3,dt/3),k4,dt/6);
    n.q=normalizeQ(n.q); n.mass=std::max(8000.0,n.mass); return n;
}

double gamma(const State&s){ double r=norm(s.r), v=std::max(1e-3,norm(s.v)); return std::asin(std::clamp(dot(s.r,s.v)/(r*v),-1.0,1.0)); }

struct EvalLog { double t,h,v,g,m,th,al,be,ba,yc,qdyn,heat,load; };

double evaluate(Individual& ind, const State& x0, const HimesVehicle& hv, const Target& tgt, double dt, std::vector<EvalLog>* log=nullptr){
    State s=x0; double J=0;
    for(size_t i=0;i<ind.genes.size();++i){
        Control u=clampControl(ind.genes[i]);
        double h=norm(s.r)-kEarthRadius, sp=std::max(1.0,norm(s.v)), dens=rho(h);
        double qdyn=0.5*dens*sp*sp;
        double heat=1.83e-4*std::sqrt(dens)*std::pow(sp,3.0); // 簡易熱流束
        double load=norm(gravityAccel(s.r))/9.80665 + qdyn/(60000.0); // 簡易荷重

        // 制約ペナルティ（明示）
        if(qdyn>50000) J += std::pow((qdyn-50000)/5000,2);
        if(heat>1.2e6) J += std::pow((heat-1.2e6)/2e5,2);
        if(load>6.0) J += std::pow((load-6.0)/0.5,2);

        if(log) log->push_back({i*dt,h,sp,gamma(s)*kRad2Deg,s.mass,u.thrustFrac,u.alpha*kRad2Deg,u.beta*kRad2Deg,u.bank*kRad2Deg,u.yawCmd*kRad2Deg,qdyn,heat,load});
        s=rk4(s,u,hv,dt);
        if((norm(s.r)-kEarthRadius)<-100) J+=1e8;
    }
    J += std::pow((norm(s.r)-kEarthRadius-tgt.altitude)/1000.0,2);
    J += std::pow((norm(s.v)-tgt.speed)/50.0,2);
    J += std::pow((gamma(s)-tgt.gamma)/(2*kDeg2Rad),2);
    J += 0.0003*(x0.mass-s.mass);
    ind.fitness=J; return J;
}

} // namespace

int main(){
    HimesVehicle hv; Target tgt{40000,1500,2*kDeg2Rad};
    State x0{{kEarthRadius+12000,0,0},{750,0,0},{0,0,0},{1,0,0,0},hv.mass};
    constexpr int H=120,P=64,G=400,E=8; constexpr double dt=0.5; constexpr double time_limit_s=4.0;

    std::mt19937_64 rng((uint64_t)std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> u01(0,1); std::normal_distribution<double> n01(0,1);
    auto randCtrl=[&](){ return Control{u01(rng),(-5+20*u01(rng))*kDeg2Rad,(-3+6*u01(rng))*kDeg2Rad,(-45+90*u01(rng))*kDeg2Rad,(-8+16*u01(rng))*kDeg2Rad}; };

    std::vector<Individual> pop(P);
    for(auto& i:pop){ i.genes.resize(H); for(auto& g:i.genes) g=randCtrl(); }

    auto t0=std::chrono::steady_clock::now();
    for(int gen=0; gen<G; ++gen){
        // OpenMP並列評価
        #pragma omp parallel for
        for(int i=0;i<P;++i) evaluate(pop[i],x0,hv,tgt,dt,nullptr);

        std::sort(pop.begin(),pop.end(),[](auto&a,auto&b){return a.fitness<b.fitness;});
        std::vector<Individual> next(pop.begin(),pop.begin()+E);
        auto tour=[&]()->const Individual&{ int a=(int)(u01(rng)*P)%P,b=(int)(u01(rng)*P)%P; return pop[a].fitness<pop[b].fitness?pop[a]:pop[b]; };
        while((int)next.size()<P){
            const auto&p1=tour(); const auto&p2=tour(); Individual c; c.genes.resize(H);
            for(int k=0;k<H;++k){ c.genes[k]=(u01(rng)<0.5)?p1.genes[k]:p2.genes[k]; if(u01(rng)<0.15){ c.genes[k].thrustFrac=std::clamp(c.genes[k].thrustFrac+0.12*n01(rng),0.0,1.0); c.genes[k].alpha+=2.5*kDeg2Rad*n01(rng); c.genes[k].beta+=1.5*kDeg2Rad*n01(rng); c.genes[k].bank+=4*kDeg2Rad*n01(rng); c.genes[k].yawCmd+=2*kDeg2Rad*n01(rng);} }
            next.push_back(std::move(c));
        }
        pop.swap(next);
        if(gen%20==0) std::cout<<"Generation "<<gen<<" | best="<<pop.front().fitness<<"\n";

        double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
        if(elapsed>time_limit_s){ std::cout<<"Time limit reached at generation "<<gen<<"\n"; break; }
    }

    std::sort(pop.begin(),pop.end(),[](auto&a,auto&b){return a.fitness<b.fitness;});
    std::vector<EvalLog> log; evaluate(pop.front(),x0,hv,tgt,dt,&log);

    std::filesystem::create_directories("results");
    std::ofstream ofs("results/best_trajectory.csv");
    ofs<<"time_s,altitude_m,speed_mps,gamma_deg,mass_kg,thrust_frac,alpha_deg,beta_deg,bank_deg,yaw_deg,qdyn_pa,heat_wm2,load_g\n";
    for(const auto& r:log) ofs<<r.t<<","<<r.h<<","<<r.v<<","<<r.g<<","<<r.m<<","<<r.th<<","<<r.al<<","<<r.be<<","<<r.ba<<","<<r.yc<<","<<r.qdyn<<","<<r.heat<<","<<r.load<<"\n";
    std::cout<<"Saved: results/best_trajectory.csv\n";
}
