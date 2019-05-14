#include "Attack.hpp"
#include "Map.hpp"
#include "Math.hpp"
#include "SafeOp.hpp"
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
    static int cacheRef = 0, cacheMiss = 0;
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
        Point cur = estimate(i);
        if(dist(old, cur) >
           CONST::human_velocity * 0.2)
            ++cacheMiss;
        ++cacheRef;
    }
    debug() << "sample miss = "
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
        if(MapHelper::noWall(base, p)) {
            SafeOp::fire(num, p);
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
    FT mval = 1e20;
    bool flag = false;
    for(int i = 0; i < sampleCnt; ++i) {
        Point pos =
            MapHelper::clampMap(Math::sampleCircle(
                dst, 0.0, CONST::explode_radius));
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
        SafeOp::meteor(num, mp);
    }
    return flag;
}
using EvalFunc = std::function<FT(const Human&)>;
void doAttack(int id, const Human& cur,
              const Point& cp, const EvalFunc& eval,
              std::vector<Meteor>& oldMeteors) {
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
        FT cdis = dist(cp, getUnit(j).position);
        if(cdis < mdis)
            mdis = cdis, cobj = j;
    }
    if(canFire(cur))
        if(obj == -1 || !tryFire(cp, id, obj)) {
            if(cobj == -1 || !tryFire(cp, id, cobj)) {
                bool flag = true;
                if(facSiz() == 2) {
                    Point np = getMap().birth_places
                                   [getMyFac() ^ 1]
                                   [Math::getRng()() %
                                    humanSiz()];
                    if(MapHelper::noWall(cp, np)) {
                        SafeOp::fire(id, np);
                        flag = false;
                    }
                }
                if(flag) {
                    Point dst = MapHelper::maxFire(cp);
                    SafeOp::fire(id, dst);
                }
            }
        }
    if(canMeteor(cur)) {
        if(obj == -1 ||
           !tryMeteor(cp, id, getUnit(obj).position,
                      oldMeteors)) {
            if(cobj == -1 ||
               !tryMeteor(cp, id,
                          getUnit(cobj).position,
                          oldMeteors)) {
                if(cobj != -1) {
                    Point op = getUnit(cobj).position;
                    FT cdis = dist(op, cp);
                    const FT offset =
                        CONST::flash_distance +
                        CONST::human_velocity *
                            CONST::meteor_delay;
                    if(cdis > offset) {
                        tryMeteor(cp, id,
                                  normalize(op - cp) *
                                      (cdis - offset),
                                  oldMeteors);
                    }
                }
            }
        }
    }
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
                return 1e20;
            return em.hp * 2.0 + dist(em.position, cp);
        };
        if(task[i] == "trans") {
            func = [&](const Human& em) {
                if(!MapHelper::noWall(em.position, cp))
                    return 1e20;
                return dist(em.position, cp);
            };
        }
        if(task[i] == "help") {
            func = [&](const Human& em) {
                if(!MapHelper::noWall(em.position, cp))
                    return 1e20;
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
                    return 1e20;
                const Crystal& ref =
                    getCrystal(getMyFac());
                int bel = ref.belong;
                if(bel == em.number)
                    return -1e20;
                return em.hp * 2.0 +
                    dist(em.position, cp) +
                    dist(em.position, ref.position) *
                    5.0;
            };
        }
        doAttack(i, cur, cp, func, oldMeteors);
    }
}
