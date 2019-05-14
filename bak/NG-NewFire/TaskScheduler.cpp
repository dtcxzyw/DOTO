#include "TaskScheduler.hpp"
#include "Map.hpp"
#include <algorithm>
#include <set>
FT minDist(const Point& pos, bool self) {
    FT val = 1e20;
    for(int i = 0; i < facSiz(); ++i) {
        if(self ^ (i == getMyFac()))
            continue;
        for(int j = 0; j < humanSiz(); ++j) {
            if(getUnit(i, j).hp > 0) {
                val = std::min(
                    val,
                    MapHelper::estimateDis(
                        pos, getUnit(i, j).position));
            }
        }
    }
    return val;
}
FT evalCrystal(const Crystal& ref) {
    FT res = minDist(ref.position, false) -
        minDist(ref.position, true);
    if(ref.faction == getMyFac())
        return res;
    if(ref.belong != -1 &&
       getFac(ref.belong) != getMyFac())
        res -= panic *
            (getUnit(ref.belong).inv_time + 10);
    return res - MapHelper::estimateDis(ref.position,
                                        getMyTarget());
}
int closest(const std::set<int>& idle,
            const Point& p) {
    int mid = -1;
    FT mdis = 1e20;
    for(int id : idle) {
        FT cdis = dist(p, getMyUnit(id).position);
        if(cdis < mdis)
            mdis = cdis, mid = id;
    }
    return mid;
}
std::vector<Task> calcTask(int& mid) {
    std::set<int> idle, bak;
    int alive = 0;
    for(int i = 0; i < humanSiz(); ++i)
        if(getMyUnit(i).hp > 0)
            idle.insert(i), ++alive;
    bak = idle;
    std::vector<Task> mops;
    mid = -1;
    for(int i = 0; i < facSiz(); ++i) {
        if(getCrystal(i).belong != -1 &&
           getFac(getCrystal(i).belong) ==
               getMyFac()) {
            mid = getNum(getCrystal(i).belong);
        }
    }
    int cid = -1;
    {
        FT mval = -1e20;
        for(int i = 0; i < facSiz(); ++i)
            if(getCrystal(i).faction != getMyFac()) {
                FT val = evalCrystal(getCrystal(i));
                if(val > mval)
                    mval = val, cid = i;
            }
    }
    Point cryPos = cid == -1 ?
        Point(0.0, 0.0) :
        getMap()
            .crystal_places[getCrystal(cid).faction];
    if(idle.count(mid)) {
        {
            Task mop;
            mop.id.push_back(mid);
            mop.dst = getMyTarget();
            mop.radius = CONST::target_radius;
            mop.taskName = "trans";
            mops.push_back(mop);
            idle.erase(mid);
        }
        if(cid != -1) {
            Point mp = getMyUnit(mid).position;
            FT mtdis = dist(mp, getMyTarget());
            bool flag =
                mtdis > minDist(getMyTarget(), false);
            Task mop;
            std::vector<int> app;
            for(int id : idle) {
                Point cp = getMyUnit(id).position;
                FT mdis = 1e20;
                for(const Point& bp :
                    getMap().bonus_places)
                    mdis =
                        std::min(mdis, dist(cp, bp));
                if(mdis < 3.0 * CONST::bonus_radius)
                    continue;
                if(mtdis > dist(cp, cryPos) || flag)
                    app.push_back(id);
            }
            std::sort(
                app.begin(), app.end(),
                [&](int a, int b) {
                    return dist(getMyUnit(a).position,
                                mp) <
                        dist(getMyUnit(b).position,
                             mp);
                });
            if(app.size() > 2)
                app.resize(2);
            for(int id : app)
                mop.id.push_back(id), idle.erase(id);
            mop.taskName = "help";
            mop.radius = 3.0 * vel;
            mop.dst = mp;
            mops.push_back(mop);
        }
    }
    // bonus
    {
        std::vector<Point> bp = getMap().bonus_places;
        auto eval = [&](const Point& p) {
            FT val = 1e20;
            for(int id : idle)
                val = std::min(
                    val,
                    dist(getMyUnit(id).position, p));
            return val;
        };
        std::sort(bp.begin(), bp.end(),
                  [&](const Point& a, const Point& b) {
                      return eval(a) < eval(b);
                  });
        for(size_t i = 0; i < bp.size(); ++i) {
            int tid = -1;
            FT mdis = 1e20;
            for(int id : idle) {
                Point cp = getMyUnit(id).position;
                FT cdis = dist(cp, bp[i]);
                if(cid != -1 &&
                   dist(cp, getCrystal(cid).position) <
                       std::min(cdis, linkDis))
                    continue;
                if(cdis < mdis)
                    mdis = cdis, tid = id;
            }
            if(tid == -1)
                break;
            idle.erase(tid);
            Task mop;
            mop.id.push_back(tid);
            mop.dst = bp[i];
            mop.radius = CONST::bonus_radius;
            mop.taskName = "bonus";
            mops.push_back(mop);
        }
    }
    if(bak.count(mid)) {
        for(int sid : idle) {
            FT mdis = 0.0;
            int fid = sid;
            for(int pid : idle) {
                FT cdis =
                    dist(getMyUnit(sid).position,
                         getMyUnit(pid).position);
                if(cdis > mdis)
                    mdis = cdis, fid = pid;
            }
            Task mop;
            if(mdis < linkDis) {
                mop.dst = cryPos;
                mop.radius = CONST::ball_radius;
            } else {
                mop.dst = getMyUnit(fid).position;
                mop.radius = 3.0 * vel;
            }
            mop.taskName = "steal";
            mop.id.push_back(sid);
            mops.push_back(mop);
        }
    } else {
        Task mop;
        mop.id.assign(idle.begin(), idle.end());
        if(cid != -1) {
            mop.taskName = "steal";
            mop.dst = getCrystal(cid).position;
            mop.radius = CONST::ball_radius;
        } else {
            mop.taskName = "guard";
            mop.radius = 3.0 * vel;
            mop.dst =
                getMap().crystal_places[getMyFac()];
        }
        mops.push_back(mop);
    }
    int sub = 0;
    for(const Task& mop : mops)
        for(int id : mop.id)
            bak.erase(id), ++sub;
    if(bak.size() || sub != alive)
        error() << "Bad task schedule:tot=" << alive
                << " but remain=" << bak.size()
                << " sub=" << sub << std::endl;
    return mops;
}
