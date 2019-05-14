#include "Attack.hpp"
#include "Map.hpp"
#include "Math.hpp"
#include "SafeOp.hpp"
#include <cmath>
#include <functional>
std::vector<std::pair<Point, Point>> samples;
Point estimate(int id) {
    if(samples.size() == getLogic().humans.size())
        return samples[id].first - samples[id].second;
    return Point(0, 0);
}
void updateSamples(bool forceRep) {
    size_t hsiz = getLogic().humans.size();
    samples.resize(hsiz);
    static int cacheRef = 1, cacheMiss = 0;
    for(size_t i = 0; i < hsiz; ++i) {
        Point cp = getUnit(i).position;
        Point old = estimate(i);
        if(!forceRep &&
           dist(cp, samples[i].first) <
               CONST::human_velocity * 1.1)
            samples[i].second = samples[i].first;
        else
            samples[i].second = cp;
        samples[i].first = cp;
        if(getFac(i) != getMyFac()) {
            Point cur = estimate(i);
            if(dist(old, cur) >
               CONST::human_velocity * 0.2)
                ++cacheMiss;
            ++cacheRef;
        }
    }
    info() << "sample miss = "
           << 100.0 * cacheMiss / cacheRef << "%"
           << std::endl;
}
Point predict(const Point& A, const Point& B,
              const Point& vb) {
    if(fabs(Math::cross(A - B, vb)) < 1e-8)
        return B;
    FT l = Math::bias, r = 1000.0;
    while(r - l >= 0.01) {
        FT m = (l + r) * 0.5;
        Point bp = B + vb * m;
        Point va = normalize(A - B);
        Point ap = A +
            va * (CONST::splash_radius +
                  CONST::fireball_velocity * m);
        if(SameSide(ap, A, makeLine(bp, B)))
            l = m;
        else
            r = m;
    }
    return B + vb * l;
}
bool tryFire(const Point& base, int num, int dst) {
    Point pdst = getUnit(dst).position;
    if(dist(pdst, base) > 2.0 *
           CONST::frames_per_second *
           CONST::fireball_velocity) {
        Point dir = estimate(dst);
        Point cp = predict(base, pdst, dir);
        if(MapHelper::noWall(base, cp) &&
           MapHelper::noWall(pdst, cp))
            pdst = cp;
    }
    const int sampleCnt = 1024;
    for(int i = 0; i < sampleCnt; ++i) {
        Point p = Math::sampleCircle(
            pdst, 0.0,
            std::min(2.0 * CONST::fireball_radius,
                     dist(pdst, base) /
                         CONST::fireball_velocity *
                         CONST::human_velocity));
        if(MapHelper::canFireAt(base, p) &&
           MapHelper::noWall(base, p)) {
            SafeOp::fire(num, p, SRCLOC);
            return true;
        }
    }
    return false;
}
FT sharedArea(const Point& a, const Point& b) {
    FT d = dist(a, b) * 0.5;
    if(d + Math::bias > CONST::explode_radius)
        return 0.0;
    FT angle = 2.0 * acos(d / CONST::explode_radius);
    return (angle - sin(angle)) *
        CONST::explode_radius * CONST::explode_radius;
}
bool tryMeteor(const Point& src, int num,
               const Point& dst,
               std::vector<Meteor>& oldMeteors) {
    const int sampleCnt = 1024;
    Point mp;
    FT mval = Math::inf;
    bool flag = false;
    for(int i = 0; i < sampleCnt; ++i) {
        Point pos = MapHelper::clampMap(
            Math::sampleCircleGauss(
                dst, CONST::explode_radius));
        if(dist(pos, src) <
           CONST::meteor_distance - Math::bias) {
            FT sv = 0.0;
            for(const Meteor& meteor : oldMeteors)
                sv +=
                    sharedArea(meteor.position, pos) *
                    meteor.last_time;
            if(sv < mval)
                mval = sv, mp = pos;
            flag = true;
        }
    }
    if(flag) {
        oldMeteors.emplace_back(
            mp.x, mp.y, CONST::meteor_delay, num);
        SafeOp::meteor(num, mp, SRCLOC);
    }
    return flag;
}
FT getAngle(FT angle) {
    static FT ang = 0.0;
    const int cycle = 6;
    ang += angle / cycle;
    FT cur = fmod(ang, 2.0 * angle);
    if(cur > angle)
        cur = angle - (cur - angle);
    return cur - angle * 0.5;
}
using EvalFunc = std::function<FT(const Human&)>;
int getClosest(const Point& pos, FT minDis) {
    int sobj = -1;
    FT mdis = Math::inf;
    int hsiz = getLogic().humans.size();
    for(int i = 0; i < hsiz; ++i) {
        FT cdis = dist(getUnit(i).position, pos);
        if(getFac(i) == getMyFac() ||
           getUnit(i).hp <= 0 || cdis < minDis)
            continue;
        if(cdis < mdis)
            mdis = cdis, sobj = i;
    }
    return sobj;
}
void doFire(int id, const Point& cp,
            const std::string& task, int obj,
            int cobj) {
    if(task == "bonus") {
        Point pos(0.0, 0.0);
        FT mdis = Math::inf;
        for(const Point& p : getMap().bonus_places) {
            FT cdis = dist(p, cp);
            if(cdis < mdis)
                mdis = cdis, pos = p;
        }
        int sobj = getClosest(pos, -1.0);
        if(sobj != -1 &&
           dist(cp, pos) < CONST::bonus_radius) {
            Point op = getUnit(sobj).position;
            if(dist(op, pos) < CONST::bonus_radius) {
                Point nop = op + estimate(sobj) * 2.0;
                if(dist(cp, nop) >
                       CONST::splash_radius &&
                   MapHelper::canFireAt(cp, nop))
                    SafeOp::fire(id, nop, SRCLOC);
                return;
            }
        }
    }
    if(obj != -1 && tryFire(cp, id, obj))
        return;
    if(cobj != -1 && tryFire(cp, id, cobj))
        return;
    {
        int sobj =
            getClosest(cp, CONST::splash_radius);
        if(sobj != -1) {
            FT mang = Math::pi / 3.0;
            FT ang = getAngle(mang);
            Point dst = rotate(cp, ang,
                               getUnit(sobj).position);
            if(MapHelper::noWall(cp, dst) &&
               MapHelper::canFireAt(cp, dst)) {
                SafeOp::fire(id, dst, SRCLOC);
                return;
            }
        }
    }
    if(facSiz() == 2) {
        Point np =
            getMap().birth_places[getMyFac() ^ 1]
                                 [Math::getRng()() %
                                  humanSiz()];
        if(MapHelper::canFireAt(cp, np) &&
           MapHelper::noWall(cp, np)) {
            SafeOp::fire(id, np, SRCLOC);
            return;
        }
    }
    Point dst = MapHelper::maxFire(cp);
    if(MapHelper::canFireAt(cp, dst))
        SafeOp::fire(id, dst, SRCLOC);
}
void doMeteor(int id, const Point& cp,
              const std::string& task, int obj,
              int cobj,
              std::vector<Meteor>& oldMeteors) {
    if(task == "bonus") {
        Point pos(0.0, 0.0);
        FT mdis = Math::inf;
        for(const Point& p : getMap().bonus_places) {
            FT cdis = dist(p, cp);
            if(cdis < mdis)
                mdis = cdis, pos = p;
        }
        if(dist(cp, pos) <
           CONST::meteor_distance - Math::bias) {
            SafeOp::meteor(id, pos, SRCLOC);
            return;
        }
    }
    {
        Point cryPos = getCrystal(getMyFac()).position;
        if(dist(cryPos, cp) <
               CONST::meteor_distance - Math::bias &&
           minDist(cryPos, false) <
               CONST::flash_distance) {
            SafeOp::meteor(id, cryPos, SRCLOC);
            return;
        }
    }
    if(cobj != -1) {
        Point op = getUnit(cobj).position;
        FT cdis = dist(op, cp);
        if(cdis <
               CONST::meteor_distance - Math::bias &&
           cdis > vel * 2.0) {
            bool close =
                Math::dot(cp - op, estimate(cobj)) >
                0.0;
            const FT addi = CONST::explode_radius;
            if(close && cdis > addi &&
               tryMeteor(cp, id,
                         cp + Math::scale(op - cp,
                                          cdis - addi),
                         oldMeteors))
                return;
            if(!close &&
               cdis + addi < CONST::meteor_distance -
                       Math::bias &&
               tryMeteor(cp, id,
                         cp + Math::scale(op - cp,
                                          cdis + addi),
                         oldMeteors))
                return;
        }
    }
    if(obj != -1 &&
       tryMeteor(cp, id, getUnit(obj).position,
                 oldMeteors))
        return;
    if(cobj != -1 &&
       tryMeteor(cp, id, getUnit(cobj).position,
                 oldMeteors))
        return;
}
void doAttack(int id, const Human& cur,
              const Point& cp, const EvalFunc& eval,
              std::vector<Meteor>& oldMeteors,
              const std::string& task) {
    int hsiz = getLogic().humans.size();
    int obj = -1, cobj = -1;
    FT mval = Math::inf, mdis = Math::inf;
    const FT maxDist = 2.0 * CONST::fireball_velocity *
        CONST::frames_per_second;
    for(int i = 0; i < hsiz; ++i) {
        FT cdis = dist(cp, getUnit(i).position);
        if(getFac(i) == getMyFac() ||
           getUnit(i).hp <= 0 || cdis > maxDist ||
           cdis < CONST::splash_radius)
            continue;
        FT cval = eval(getUnit(i));
        if(cval < mval)
            mval = cval, obj = i;
        if(cdis < mdis)
            mdis = cdis, cobj = i;
    }
    if(canFire(cur))
        doFire(id, cp, task, obj, cobj);
    if(canMeteor(cur))
        doMeteor(id, cp, task, obj, cobj, oldMeteors);
}
void calcAttack(int mid,
                const std::vector<Point>& nxtPos,
                const std::vector<std::string>& task) {
    std::vector<Meteor> oldMeteors;
    for(const Meteor& meteor : getLogic().meteors) {
        if(!isAnti(meteor.from_number))
            oldMeteors.push_back(meteor);
    }
    for(int i = 0; i < humanSiz(); ++i) {
        const Human& cur = getMyUnit(i);
        if(cur.hp <= 0)
            continue;
        Point cp = nxtPos[i];
        EvalFunc func = [&](const Human& em) {
            if(!MapHelper::noWall(em.position, cp))
                return Math::inf;
            const Crystal& ref =
                getCrystal(getMyFac());
            int bel = ref.belong;
            if(bel == em.number)
                return -Math::inf;
            return em.hp * 2.0 + dist(em.position, cp);
        };
        if(task[i] == "trans") {
            func = [&](const Human& em) {
                if(!MapHelper::noWall(em.position, cp))
                    return Math::inf;
                return dist(em.position, cp);
            };
        }
        if(task[i] == "help") {
            func = [&](const Human& em) {
                if(!MapHelper::noWall(em.position, cp))
                    return Math::inf;
                FT off = 0.0;
                if(mid != -1)
                    off = dist(em.position,
                               nxtPos[mid]) *
                        10.0;
                return em.hp * 2.0 +
                    dist(em.position, cp) + off;
            };
        }
        if(task[i] == "guard" || task[i] == "steal") {
            func = [&](const Human& em) {
                if(!MapHelper::noWall(em.position, cp))
                    return Math::inf;
                const Crystal& ref =
                    getCrystal(getMyFac());
                int bel = ref.belong;
                if(bel == em.number)
                    return -Math::inf;
                return em.hp * 2.0 +
                    dist(em.position, cp) +
                    dist(em.position, ref.position) *
                    5.0;
            };
        }
        doAttack(i, cur, cp, func, oldMeteors,
                 task[i]);
    }
}
