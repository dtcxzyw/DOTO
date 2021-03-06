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
    for(size_t i = 0; i < hsiz; ++i) {
        Point cp = getUnit(i).position;
        if(!forceRep &&
           dist(cp, samples[i].first) <
               CONST::fireball_velocity * 1.1)
            samples[i].second = samples[i].first;
        else
            samples[i].first = cp;
        samples[i].first = cp;
    }
}
Point predict(const Point& A, const Point& B,
              const Point& vb) {
    if(fabs(Math::cross(A - B, vb)) < 1e-8)
        return B;
    FT l = Math::bias, r = 200.0;
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
    // if(dist(pdst, base) > 15.0) {
    //   Point dir = estimate(dst);
    //   pdst = predict(base, pdst, dir);
    //}
    if(MapHelper::noWall(base, pdst)) {
        SafeOp::fire(num, pdst);
        return true;
    }
    const int sampleCnt = 1024;
    for(int i = 0; i < sampleCnt; ++i) {
        Point p = Math::sampleCircle(
            pdst, 0.0, CONST::fireball_radius);
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
                dst, 0.0, CONST::explode_radius));
        if(dist(pos, src) <
           CONST::meteor_distance - Math::bias) {
            SafeOp::meteor(num, pos);
            return true;
        }
    }
    return false;
}
using EvalFunc = std::function<FT(const Human&)>;
void doAttack(int id, const Human& cur,
              const Point& cp, const EvalFunc& eval) {
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
                Point dst = MapHelper::maxFire(cp);
                SafeOp::fire(id, dst);
            }
        }
    if(canMeteor(cur)) {
        if(obj == -1 ||
           !tryMeteor(cp, id, getUnit(obj).position)) {
            if(cobj == -1 ||
               !tryMeteor(cp, id,
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
void calcAttack(int mid,
                const std::vector<Point>& nxtPos,
                const std::vector<std::string>& task) {
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
        if(task[i] == "guard") {
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
        doAttack(i, cur, cp, func);
    }
}
