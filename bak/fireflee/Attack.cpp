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
    if(dist(pdst, base) > 15.0) {
        Point dir = estimate(dst);
        pdst = predict(base, pdst, dir);
    }
    if(MapHelper::noWall(base, pdst)) {
        SafeOp::fire(num, pdst);
        return true;
    }
    const int sampleCnt = 1024;
    for(int i = 0; i < sampleCnt; ++i) {
        Point p = Math::sampleCircle(
            pdst, CONST::fireball_radius);
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
                dst, CONST::explode_radius));
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
        Point cp = nxtPos[i];
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
                    dist(cp, hu.position) +
                    dist(hu.position, pos) * 10.0;
            }) :
            EvalFunc([&](const Human& hu) {
                return hu.hp +
                    (hu.inv_time + canFlash(hu) * 5) *
                    panic +
                    dist(cp, hu.position);
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
            if(obj == -1 || !tryFire(cp, i, obj)) {
                if(cobj == -1 ||
                   !tryFire(cp, i, cobj)) {
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
