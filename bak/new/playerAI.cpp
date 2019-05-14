#include "playerAI.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <queue>
#include <random>
#include <set>
#include <sstream>

using Clock = std::chrono::high_resolution_clock;
using FT = double;

const FT vel =
    CONST::frames_per_second * CONST::human_velocity;
const FT linkDis = vel * 7.0;
const FT closeDis = vel * 15.0;
const FT panic = 10.0;

Point operator+(const Point& lhs, const Point& rhs) {
    return Point(lhs.x + rhs.x, lhs.y + rhs.y);
}
Point operator*(const Point& v, FT k) {
    return Point(v.x * k, v.y * k);
}
Point operator-(const Point& lhs, const Point& rhs) {
    return Point(lhs.x - rhs.x, lhs.y - rhs.y);
}
FT length(const Point& v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

std::ostringstream& debug() {
    static std::ostringstream ss;
    return ss;
}

namespace Math {
    const FT bias = 1e-4;
    const FT pi = 3.14159265358979323846;
    const FT twoPi = 2.0 * pi;
    const FT pi2 = 1.57079632679489661923;
    const FT pi4 = 0.78539816339744830962;
    const FT invPi = 0.31830988618379067154;
    const FT twoInvPi = 0.63661977236758134308;
    const FT twoISqrtPi = 1.12837916709551257390;
    const FT sqrt2 = 1.41421356237309504880;
    const FT invSqrt2 = 0.70710678118654752440;
    using Rng = std::mt19937_64;
    Rng& getRng() {
        static Rng eng(
            Clock::now().time_since_epoch().count());
        return eng;
    }
    FT clampRad(FT x, FT rad) {
        return std::min(std::max(x, -rad), rad);
    }
    FT sampleGauss(FT radius) {
        std::normal_distribution<FT> distrib(0.0, 1.0);
        return clampRad(
            distrib(getRng()) / 3.0 * radius, radius);
    }
    Point sampleRing(const Point& base, FT radius) {
        std::uniform_real_distribution<FT> dang(
            0.0, 2.0 * pi);
        FT angle = dang(getRng());
        return Point(base.x + cos(angle) * radius,
                     base.y + sin(angle) * radius);
    }
    Point sampleCircle(const Point& base, FT radius) {
        std::uniform_real_distribution<FT> rang(
            0.0, radius);
        FT rad = rang(getRng());
        return sampleRing(base, rad);
    }
    Point sampleCircleGauss(const Point& base,
                            FT radius) {
        return sampleRing(base,
                          fabs(sampleGauss(radius)));
    }
    Point lerp(const Point& a, const Point& b, FT w) {
        return a * (1.0 - w) + b * w;
    }
    Point rlerp(const Point& a, const Point& b) {
        std::uniform_real_distribution<FT> urd(0.0,
                                               1.0);
        return lerp(a, b, urd(getRng()));
    }
}
Logic& getLogic() {
    return *Logic::Instance();
}
const Map& getMap() {
    return getLogic().map;
}
int facSiz() {
    return getMap().faction_number;
}
int humanSiz() {
    return getMap().human_number;
}
int getID(int faction, int num) {
    return num * facSiz() + faction;
}
const Human& getUnit(int id) {
    return getLogic().humans[id];
}
const Human& getUnit(int faction, int num) {
    return getUnit(getID(faction, num));
}
int getMyFac() {
    return getLogic().faction;
}
const Human& getMyUnit(int num) {
    return getUnit(getMyFac(), num);
}
int getFac(int id) {
    return id % facSiz();
}
int getNum(int id) {
    return id / facSiz();
}
const Crystal& getCrystal(int id) {
    return getLogic().crystal[id];
}
std::pair<Point, bool> getBonus(int id) {
    return std::make_pair(getMap().bonus_places[id],
                          getLogic().bonus[id]);
}
Point getMyTarget() {
    return getMap().target_places[getMyFac()];
}
namespace MapHelper {
    bool isWall(int x, int y) {
        if(x < 0 || y < 0 || x >= getMap().width ||
           y >= getMap().height)
            return true;
        return getMap().pixels[x][y];
    }
    bool isWall(const Point& v) {
        return isWall(v.x, v.y);
    }
    bool noWall(const Point& s, const Point& t) {
        const int maxSample = 1000;
        int sampleCnt =
            ceil(dist(s, t) / CONST::room_size * 1.1);
        sampleCnt = std::max(
            3, std::min(sampleCnt, maxSample));
        for(int i = 0; i <= sampleCnt; ++i)
            if(isWall(Math::lerp(
                   s, t,
                   i / static_cast<FT>(sampleCnt))))
                return false;
        return true;
    }
    bool isWalkWall(const Point& v, int invTime) {
        if(isWall(v))
            return true;
        for(const Meteor& m : getLogic().meteors) {
            if(getFac(m.from_number) != getMyFac() &&
               dist(m.position, v) <=
                   CONST::explode_radius +
                       Math::bias &&
               m.last_time > invTime)
                return true;
        }
        // fireball???
        return false;
    }
    FT estimateDis(const Point& a, const Point& b) {
        return dist(a, b);
    }
    Point maxFire(const Point& base) {
        Point res(0.0, 0.0);
        int mcnt = -(1 << 30);
        const int sampleCnt = 128.0;
        for(int i = 0; i < sampleCnt; ++i) {
            FT ang = Math::twoPi * i / sampleCnt;
            Point dir = Point(cos(ang), sin(ang)),
                  off = dir * CONST::room_size,
                  cur = base +
                dir * CONST::splash_radius;
            int cnt = 0;
            while(!isWall(cur)) {
                ++cnt;
                cur = cur + off;
            }
            if(cnt > mcnt)
                mcnt = cnt, res = cur - off;
        }
        return res;
    }
    Point clampMap(const Point& p) {
        using Math::bias;
        FT cx = std::max(
            bias,
            std::min(p.x, getMap().width - bias));
        FT cy = std::max(
            bias,
            std::min(p.y, getMap().height - bias));
        return Point(cx, cy);
    }
}
/*
namespace Seer {
    struct Sample {
        Point his;
        int frame;
        Sample() : frame(-1) {}
        Sample(int id) {
            frame = getLogic().frame;
            his = getUnit(id).position;
        }
        Point estimate(const Point& cur) const {
            if(frame == -1)
                return cur;
            Point off = cur - his;
            if(length(off) > CONST::human_velocity * 2)
                return cur;
            return cur + off;
        }
    };
    std::vector<Sample> samples;
    Point estimate(int id) {
        Point cur = getUnit(id).position;
        if(samples.size() == getLogic().humans.size())
            return samples[id].estimate(cur);
        return cur;
    }
    void updateSamples() {
        size_t hsiz = getLogic().humans.size();
        if(samples.size() != hsiz)
            samples.resize(hsiz);
        for(size_t i = 0; i < hsiz; ++i)
            samples[i] = Sample(i);
    }
}
*/
bool canFlash(const Human& hu) {
    if(hu.hp <= 0 || hu.flash_num <= 0 ||
       hu.flash_time > 0)
        return false;
    for(const Crystal& cry : getLogic().crystal)
        if(cry.belong == hu.number)
            return false;
    return true;
}
bool canFire(const Human& hu) {
    return hu.fire_time <= 0 && hu.hp > 0;
}
bool canMeteor(const Human& hu) {
    return hu.meteor_number > 0 && hu.hp > 0 &&
        hu.meteor_time <= 0;
}
namespace SafeOp {
    void move(int num, const Point& p) {
        if(num >= humanSiz()) {
            debug() << "move:invaild num" << std::endl;
            return;
        }
        if(MapHelper::isWall(p)) {
            debug() << "move:invalid pos" << std::endl;
            return;
        }
        Point cp = getMyUnit(num).position;
        debug() << "move " << num << " from [" << cp.x
                << " " << cp.y << "] to [" << p.x
                << " " << p.y << "]" << std::endl;
        getLogic().move(num, p);
    }
    void fire(int num, const Point& p) {
        if(num >= humanSiz()) {
            debug() << "fire:invaild num" << std::endl;
            return;
        }
        if(!canFire(getMyUnit(num))) {
            debug() << "fire:can not fire"
                    << std::endl;
            return;
        }
        debug() << "shoot " << num << " at [" << p.x
                << " " << p.y << "]" << std::endl;
        getLogic().shoot(num, p);
    }
    void flash(int num) {
        if(num >= humanSiz()) {
            debug() << "flash:invaild num"
                    << std::endl;
            return;
        }
        if(!canFlash(getMyUnit(num))) {
            debug() << "flash:can not flash"
                    << std::endl;
            return;
        }
        debug() << "flash " << num << std::endl;
        getLogic().flash(num);
    }
    void meteor(int num, const Point& p) {
        if(num >= humanSiz()) {
            debug() << "meteor:invaild num"
                    << std::endl;
            return;
        }
        if(!canMeteor(getMyUnit(num)) ||
           dist(getMyUnit(num).position, p) >
               CONST::meteor_distance) {
            debug() << "meteor:can not meteor"
                    << std::endl;
            return;
        }
        debug() << "meteor " << num << " in [" << p.x
                << " " << p.y << std::endl;
        getLogic().meteor(num, p);
    }
}
FT evalCrystal(const Crystal& ref) {
    FT mdisA = 1e20, mdisB = 1e20;
    for(int i = 0; i < facSiz(); ++i) {
        FT& val = (i == getMyFac() ? mdisA : mdisB);
        for(int j = 0; j < humanSiz(); ++j) {
            if(getUnit(i, j).hp > 0) {
                val = std::min(
                    val, MapHelper::estimateDis(
                             ref.position,
                             getUnit(i, j).position));
            }
        }
    }
    FT res = mdisB - mdisA;
    if(ref.faction == getMyFac())
        return res;
    if(ref.belong != -1)
        res -= panic *
            (getUnit(ref.belong).inv_time + 10);
    return res - MapHelper::estimateDis(ref.position,
                                        getMyTarget());
}
struct MoveOp {
    std::vector<int> id;
    Point dst;
    FT radius;
    std::string taskName;
};
void moveSome(std::set<int>& idle, const Point& dst,
              FT radius, size_t cnt, MoveOp& mop) {
    std::vector<int> team(idle.begin(), idle.end());
    std::sort(team.begin(), team.end(),
              [&](int a, int b) {
                  auto esti = [&](int x) {
                      return MapHelper::estimateDis(
                          getMyUnit(x).position, dst);
                  };
                  return esti(a) < esti(b);
              });
    for(int id : team) {
        if(mop.id.size() >= cnt)
            break;
        idle.erase(id);
        mop.id.push_back(id);
    }
    mop.dst = dst;
    mop.radius = radius;
}
int findFa(std::vector<int>& fa, int x) {
    return fa[x] == x ? x : fa[x] = findFa(fa, fa[x]);
}
void trySteal(std::set<int>& idle, MoveOp& mop) {
    size_t stealCnt = std::max(
        static_cast<int>(idle.size()) * 3 / 5, 2);
    if(idle.size() < stealCnt)
        return;
    FT mval = -1e20;
    int id = -1;
    for(int i = 0; i < facSiz(); ++i)
        if(getCrystal(i).faction != getMyFac()) {
            FT val = evalCrystal(getCrystal(i));
            if(val > mval) {
                mval = val;
                id = i;
            }
        }
    if(id != -1) {
        const Crystal& ref = getCrystal(id);
        moveSome(idle, ref.position,
                 CONST::ball_radius, stealCnt, mop);
    }
}
int countUnits(const Point& v, FT radius, bool self) {
    int hsiz = getLogic().humans.size(), res = 0;
    for(int i = 0; i < hsiz; ++i)
        if(getUnit(i).hp > 0 &&
           (self || getFac(i) != getMyFac()))
            res +=
                dist(v, getUnit(i).position) <= radius;
    return res;
}
struct Group {
    std::vector<int> myUnit, emUnit;
};
std::vector<MoveOp>
calcTask(const std::vector<Group>& g,
         std::vector<int>& fa, int& mid) {
    std::set<int> idle;
    for(int i = 0; i < humanSiz(); ++i)
        if(getMyUnit(i).hp > 0)
            idle.insert(i);
    // steal/help/trans
    mid = -1;
    Point idleP = getMyTarget();
    for(int i = 0; i < facSiz(); ++i) {
        if(getCrystal(i).belong != -1 &&
           getFac(getCrystal(i).belong) ==
               getMyFac()) {
            mid = getNum(getCrystal(i).belong);
        }
        if(getCrystal(i).faction == getMyFac())
            idleP = getCrystal(i).position;
    }
    std::vector<MoveOp> mops;
    if(mid == -1 || !idle.count(mid)) {
        // steal
        MoveOp mop;
        trySteal(idle, mop);
        if(mop.id.size()) {
            idleP = mop.dst;
            mop.taskName = "steal";
            mops.push_back(mop);
        }
    } else {
        idleP = getMyUnit(mid).position;
        // trans and help
        {
            MoveOp mop;
            idle.erase(mid);
            mop.id.push_back(mid);
            mop.dst = getMyTarget();
            mop.radius = CONST::target_radius;
            mop.taskName = "trans";
            mops.push_back(mop);
        }
        {
            int helpCnt = 2 +
                g[findFa(fa, getID(getMyFac(), mid))]
                    .emUnit.size();
            MoveOp mop;
            moveSome(idle, getMyUnit(mid).position,
                     CONST::explode_radius * 3.0,
                     helpCnt, mop);
            mop.taskName = "help";
            mops.push_back(mop);
        }
    }
    // intercept
    /*
    {
        for(const Crystal& cry : getLogic().crystal) {
            if(cry.faction == getMyFac()) {
                FT val = evalCrystal(cry);
                if(val < 0.0) {
                    MoveOp mop;
                    moveSome(idle, cry.position, 0.0,
                             getEms(cry.position) + 1,
                             mop);
                    mops.push_back(mop);
                }
            }
        }
    }
    */
    // idle
    {
        for(int uid : idle) {
            int id = -1;
            FT mval = 1e20;
            for(size_t i = 0;
                i < getLogic().bonus.size(); ++i) {
                std::pair<Point, bool> ref =
                    getBonus(i);
                if(ref.second ||
                   countUnits(ref.first,
                              CONST::bonus_radius,
                              true)) {
                    FT cval = MapHelper::estimateDis(
                        ref.first,
                        getMyUnit(uid).position);
                    if(cval < mval)
                        mval = cval, id = i;
                }
            }
            MoveOp mop;
            mop.id.push_back(uid);
            if(id != -1) {
                mop.taskName = "idle";
                mop.dst = getBonus(id).first;
                mop.radius = CONST::bonus_radius;
            } else {
                mop.taskName = "help";
                mop.dst = idleP;
                mop.radius = 0.0;
            }
            mops.push_back(mop);
        }
    }
    return mops;
}
void doMove(int num, const Point& dst) {
    if(canFlash(getMyUnit(num)) &&
       dist(dst, getMyUnit(num).position) >
           CONST::human_velocity - Math::bias)
        SafeOp::flash(num);
    SafeOp::move(num, dst);
}
Point tryMove(int num, Point dir) {
    FT rad = CONST::human_velocity;
    if(canFlash(getMyUnit(num)))
        rad = CONST::flash_distance;
    rad -= Math::bias;
    const Human& hu = getMyUnit(num);
    if(length(dir) > Math::bias) {
        if(length(dir) > rad)
            dir = dir * (rad / length(dir));
        while(length(dir) > 0.2) {
            Point cp = hu.position + dir;
            if(!MapHelper::isWalkWall(cp,
                                      hu.inv_time)) {
                doMove(num, cp);
                return cp;
            }
            dir = dir * 0.95;
        }
    }
    debug() << "Warning:no way" << std::endl;
    const int sampleCnt = 128;
    for(int i = 0; i < sampleCnt; ++i) {
        Point cp =
            Math::sampleCircleGauss(hu.position, rad);
        if(!MapHelper::isWalkWall(cp, hu.inv_time)) {
            doMove(num, cp);
            return cp;
        }
    }
    return hu.position;
}
namespace KeyMap {
    const int tileSize = 60;
    int id[tileSize][tileSize], icnt = 0;
    Point pos[tileSize * tileSize];
    std::vector<int> LUT[tileSize * tileSize];
    bool vis[tileSize][tileSize];
    const int off[24][2] = {
        { 2, 2 },   { 2, -2 },  { -2, 2 },
        { -2, -2 },  // A
        { 2, 1 },   { 2, -1 },  { 1, 2 },
        { 1, -2 },  { -2, 1 },  { -2, -1 },
        { -1, 2 },  { -1, -2 },  // B
        { 2, 0 },   { 0, 2 },   { -2, 0 },
        { 0, -2 },  // C
        { 1, 1 },   { 1, -1 },  { -1, 1 },
        { -1, -1 },  // D
        { 1, 0 },   { 0, 1 },   { -1, 0 },
        { 0, -1 }  // E
    };
    using Pos = std::pair<int, int>;
    Pos ipos[tileSize * tileSize];
    std::vector<Pos> genNxt(int i, int j) {
        int sid = id[i][j];
        std::vector<Pos> res;
        res.reserve(24);
        for(int k = 0; k < 24; ++k) {
            int cx = i + off[k][0], cy = j + off[k][1];
            if(cx >= 0 && cx < tileSize && cy >= 0 &&
               cy < tileSize) {
                int cid = id[cx][cy];
                if(cid && MapHelper::noWall(pos[sid],
                                            pos[cid]))
                    res.emplace_back(cx, cy);
            }
        }
        return res;
    }
    struct Info {
        Pos u, dir;
        FT w;
        Info(const Pos& u, const Pos& dir, FT w)
            : u(u), dir(dir), w(w) {}
        bool operator<(const Info& rhs) const {
            return w > rhs.w;
        }
    };
    FT dist(const Pos& a, const Pos& b) {
        FT dx = a.first - b.first,
           dy = a.second - b.second;
        return sqrt(dx * dx + dy * dy);
    }
    void SSSP(std::vector<int>& A, const Pos& src) {
        std::priority_queue<Info> heap;
        heap.emplace(src, src, 0.0);
        while(heap.size()) {
            Pos u = heap.top().u;
            Pos dir = heap.top().dir;
            FT w = heap.top().w;
            heap.pop();
            if(vis[u.first][u.second])
                continue;
            vis[u.first][u.second] = true;
            A[id[u.first][u.second]] =
                id[dir.first][dir.second];
            for(const Pos& p :
                genNxt(u.first, u.second))
                heap.emplace(p, u, w + dist(u, p));
        }
    }
    void load(int id) {
        if(LUT[id].empty()) {
            memset(vis, 0, sizeof(vis));
            LUT[id].resize(icnt + 1);
            SSSP(LUT[id], ipos[id]);
        }
    }
    void init() {
        for(int i = 0; i < tileSize; ++i) {
            FT cx = static_cast<FT>(i) *
                getMap().width / tileSize;
            for(int j = 0; j < tileSize; ++j) {
                FT cy = static_cast<FT>(j) *
                    getMap().height / tileSize;
                if(!MapHelper::isWall(cx, cy)) {
                    id[i][j] = ++icnt;
                    pos[icnt] = Point(cx, cy);
                    ipos[icnt] = Pos(i, j);
                }
            }
        }
    }
    int getShortest(const Point& v) {
        int id = 0;
        FT mdis = 1e20;
        for(int i = 1; i <= icnt; ++i) {
            FT cdis = dist(v, pos[i]);
            if(cdis < mdis &&
               MapHelper::noWall(v, pos[i]))
                mdis = cdis, id = i;
        }
        return id;
    }
    Point plan(const Point& src, const Point& dst) {
        int sid = getShortest(src);
        if(sid == 0)
            return Point(0, 0);
        debug() << "close src (" << src.x << " "
                << src.y << ") is (" << pos[sid].x
                << " " << pos[sid].y << ")"
                << std::endl;

        int did = getShortest(dst);
        if(did == 0)
            return Point(0, 0);
        debug() << "close dst (" << dst.x << " "
                << dst.y << ") is (" << pos[did].x
                << " " << pos[did].y << ")"
                << std::endl;
        if(sid == did)
            return dst - src;
        load(did);
        Point nxt = pos[LUT[did][sid]];
        debug() << "nxt " << nxt.x << " " << nxt.y
                << " " << std::endl;
        return nxt - src;
    }
}
using PosInfo = std::pair<int, Point>;
std::vector<PosInfo> teamMove(const MoveOp& op) {
    std::vector<PosInfo> res;
    if(op.id.empty())
        return res;
    for(int id : op.id) {
        debug() << "unit " << id << " task "
                << op.taskName << std::endl;
        const Human& hu = getMyUnit(id);
        if(MapHelper::noWall(hu.position, op.dst))
            res.emplace_back(
                getID(getMyFac(), id),
                tryMove(id, op.dst - hu.position));
        else {
            Point dir =
                KeyMap::plan(hu.position, op.dst);
            res.emplace_back(getID(getMyFac(), id),
                             tryMove(id, dir));
        }
    }
    return res;
}
bool tryFire(const Point& base, int num,
             const Point& dst) {
    const int sampleCnt = 32;
    for(int i = 0; i < sampleCnt; ++i) {
        Point p = MapHelper::clampMap(
            Math::sampleCircleGauss(
                dst, CONST::fireball_radius));
        if(MapHelper::noWall(base, p)) {
            SafeOp::fire(num, p);
            return true;
        }
    }
    return false;
}
bool tryMeteor(const Point& src, int num,
               const Point& dst) {
    const int tryCnt = 128;
    for(int i = 0; i < tryCnt; ++i) {
        Point pos =
            MapHelper::clampMap(Math::sampleCircle(
                dst, CONST::explode_radius * 3.0));
        if(dist(pos, src) <
           CONST::meteor_distance - Math::bias) {
            SafeOp::meteor(num, pos);
            return true;
        }
    }
    return false;
}
void calcAttack(int mid, std::vector<int>& fa,
                const std::vector<Point>& nxtPos) {
    for(int i = 0; i < humanSiz(); ++i) {
        const Human& cur = getMyUnit(i);
        if(cur.hp <= 0)
            continue;
        int id = getID(getMyFac(), i);
        Point cp = nxtPos[id];
        bool protect = i != mid && mid != -1 &&
            findFa(fa, getID(getMyFac(), mid)) ==
                findFa(fa, id);
        using EvalFunc =
            std::function<FT(const Human&)>;
        EvalFunc eval = protect ?
            EvalFunc([&](const Human& hu) {
                Point pos = getMyUnit(mid).position;
                return hu.hp +
                    (hu.inv_time + canFlash(hu) * 5) *
                    panic +
                    dist(cp, hu.position) * 0.5 +
                    dist(hu.position, pos) * 10.0;
            }) :
            EvalFunc([&](const Human& hu) {
                return hu.hp +
                    (hu.inv_time + canFlash(hu) * 5) *
                    panic +
                    dist(cp, hu.position) * 0.5;
            });
        const int hsiz = getLogic().humans.size();
        int obj = -1, cobj = -1;
        FT mval = 1e20, mdis = 1e20;
        for(int j = 0; j < hsiz; ++j) {
            if(getFac(j) == getMyFac() ||
               getUnit(j).hp <= 0)
                continue;
            FT cval = eval(getUnit(j));
            if(cval < mval)
                mval = cval, obj = j;
            FT cdis = dist(cur.position,
                           getUnit(j).position);
            if(cdis < mdis)
                mdis = cdis, cobj = j;
        }
        if(canFire(cur))
            if(obj == -1 ||
               !tryFire(cp, i,
                        getUnit(obj).position)) {
                if(cobj == -1 ||
                   !tryFire(cp, i,
                            getUnit(cobj).position)) {
                    Point dst = MapHelper::maxFire(cp);
                    SafeOp::fire(i, dst);
                }
            }
        if(canMeteor(cur)) {
            if(obj == -1 ||
               !tryMeteor(cp, i,
                          getUnit(obj).position)) {
                if(cobj == -1 ||
                   !tryMeteor(
                       cp, i,
                       getUnit(cobj).position)) {
                    /*
                    Point pos = Math::sampleCircle(
                        cp, CONST::meteor_distance -
                            Math::bias);
                    getLogic().meteor(
                        i, MapHelper::clampMap(pos));
                    */
                }
            }
        }
    }
}
void debugTime(const std::string& name,
               Clock::time_point& ts) {
    Clock::time_point cur = Clock::now();
    debug() << name << " time = "
            << std::chrono::duration_cast<
                   std::chrono::milliseconds>(cur - ts)
                   .count()
            << " ms" << std::endl;
    ts = cur;
}
void playerAI() {
    debug().str("");
    if(getLogic().frame == 1)
        KeyMap::init();
    Clock::time_point now = Clock::now();
    int hsiz = getLogic().humans.size();
    std::vector<int> fa(hsiz);
    for(int i = 0; i < hsiz; ++i)
        fa[i] = i;
    for(int i = 0; i < hsiz; ++i) {
        if(getUnit(i).hp <= 0)
            continue;
        Point sp = getUnit(i).position;
        for(int j = i + 1; j < hsiz; ++j) {
            if(getUnit(j).hp <= 0 ||
               dist(sp, getUnit(j).position) > linkDis)
                continue;
            int fu = findFa(fa, i), fv = findFa(fa, j);
            if(fu != fv)
                fa[fu] = fv;
        }
    }
    std::vector<Group> groups(hsiz);
    for(int i = 0; i < hsiz; ++i)
        if(getUnit(i).hp > 0) {
            Group& addg = groups[findFa(fa, i)];
            (getFac(i) == getMyFac() ? addg.myUnit :
                                       addg.emUnit)
                .push_back(i);
        }
    debugTime("group", now);
    int mid = -1;
    std::vector<MoveOp> trans =
        calcTask(groups, fa, mid);
    debugTime("task", now);
    std::vector<Point> nxtPos(hsiz);
    for(int i = 0; i < hsiz; ++i)
        nxtPos[i] = getUnit(i).position;
    for(const MoveOp& t : trans) {
        std::vector<PosInfo> res = teamMove(t);
        for(const PosInfo& info : res)
            nxtPos[info.first] = info.second;
    }
    debugTime("move", now);
    calcAttack(mid, fa, nxtPos);
    debugTime("attack", now);
    getLogic().debug(debug().str());
    // Seer::updateSamples();
}
