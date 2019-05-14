#include "TaskScheduler.hpp"
#include "Map.hpp"
#include <algorithm>
#include <set>
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
    if(ref.belong != -1 &&
       getFac(ref.belong) != getMyFac())
        res -= panic *
            (getUnit(ref.belong).inv_time + 10);
    return res - MapHelper::estimateDis(ref.position,
                                        getMyTarget());
}
void moveSome(std::set<int>& idle, const Point& dst,
              FT radius, size_t sum, Task& mop) {
    std::vector<int> team(idle.begin(), idle.end());
    std::sort(
        team.begin(), team.end(), [&](int a, int b) {
            auto esti = [&](int x) {
                return getMyUnit(x).hp * 2 -
                    MapHelper::estimateDis(
                           getMyUnit(x).position, dst);
            };
            return esti(a) > esti(b);
        });
    for(int id : team) {
        if(mop.id.size() >= sum)
            break;
        idle.erase(id);
        mop.id.push_back(id);
    }
    mop.dst = dst;
    mop.radius = radius;
}
void trySteal(std::set<int>& idle, Task& mop) {
    size_t stealCnt = std::max(idle.size() * 3 / 5,
                               static_cast<size_t>(2));
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
           (self || getFac(i) != getMyFac()) &&
           dist(v, getUnit(i).position) <= radius)
            res += getUnit(i).hp;
    return res;
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
        size_t end = humanSiz() * 2 / 5;
        for(size_t i = 0; i < bp.size() && i < end;
            ++i) {
            int tid = -1;
            FT mdis = 1e20;
            for(int id : idle) {
                if(id == mid)
                    continue;
                FT cdis = dist(getMyUnit(id).position,
                               bp[i]);
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
    // steal/help/trans
    if(mid == -1 || !idle.count(mid)) {
        // steal
        Task mop;
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
            Task mop;
            idle.erase(mid);
            mop.id.push_back(mid);
            mop.dst = getMyTarget();
            mop.radius = CONST::target_radius;
            mop.taskName = "trans";
            mops.push_back(mop);
        }
        {
            int helpCnt = std::max(
                1, countUnits(getMyUnit(mid).position,
                              closeDis, false) *
                    14 / (10 * CONST::human_hp));
            Task mop;
            moveSome(idle, getMyUnit(mid).position,
                     vel * 3.0, helpCnt, mop);
            mop.taskName = "help";
            mops.push_back(mop);
        }
    }
    // guard
    {
        const Crystal& ref = getCrystal(getMyFac());
        Task mop;
        mop.radius = 3.0 * vel;
        mop.dst = ref.position;
        mop.taskName = "guard";
        mop.id.assign(idle.begin(), idle.end());
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
