#include "Sampler.hpp"
#include "Analyzer.hpp"
#include <omp.h>
#include <queue>

struct FixedSizeArray final {
    FT val[16];
    int siz;
    FixedSizeArray() : siz(0) {}
    void pushBack(FT v) {
        if(siz == 16) {
            ERROR << "overflow" << std::endl;
            return;
        }
        val[siz++] = v;
    }
    bool operator<(const FixedSizeArray& rhs) const {
        int end = std::min(siz, rhs.siz);
        for(int i = 0; i < end; ++i)
            if(fabs(val[i] - rhs.val[i]) > Math::bias)
                return val[i] < rhs.val[i];
        return false;
    }
};

template <typename Input>
EvalFunc<Input> bool2val(const PredFunc<Input>& func) {
    return [func](const Input& in) { return func(in).test() ? 1.0 : -1.0; };
}

PointSampler::PointSampler(const PointGenerator& gen) : mGen(gen) {}
PointSampler& PointSampler::estimate(const PointEvalFunc& eval) {
    mEvals.push_back(eval);
    return *this;
}
PointSampler& PointSampler::request(const PointPredFunc& req) {
    mReqs.push_back(req);
    return *this;
}
PointSampler& PointSampler::optional(const PointPredFunc& test) {
    mEvals.push_back(bool2val(test));
    return *this;
}
PointSampler& PointSampler::interest(const std::vector<Point>& hots) {
    for(Point cp : hots) {
        bool flag = true;
        for(const auto& req : mHotReqs)
            if(!req(cp).test()) {
                flag = false;
                break;
            }
        if(flag)
            mHots.push_back(cp);
    }
    return *this;
}
PointSampler& PointSampler::hotRequest(const PointPredFunc& req) {
    mHotReqs.push_back(req);
    return *this;
}
Point PointSampler::sample(size_t sampleCnt,
                           const std::function<Point()>& expect) const {
    auto apply = [this](const Point& cp, Point& res, FixedSizeArray& mval) {
        for(const auto& p : mReqs)
            if(!p(cp).test())
                return;
        FixedSizeArray cval;
        for(const auto& p : mEvals) {
            FT val = p(cp).get();
            if(val == Math::nan)
                return;
            cval.pushBack(val);
            if(cval < mval)
                return;
        }
        if(res == Math::invalidPos || mval < cval)
            mval = cval, res = cp;
    };
    Point res = Math::invalidPos;
    FixedSizeArray mval;
    for(Point cp : mHots) {
        apply(cp, res, mval);
        if(mEvals.empty() && res == cp)
            return cp;
    }
    int coreNum = std::max(1, omp_get_num_procs() / 2);
#pragma omp parallel num_threads(coreNum)
    {
        Point pres = res;
        FixedSizeArray pval = mval;
        for(size_t i = 0; i < sampleCnt; ++i) {
            Point cp = mGen(i, sampleCnt);
            apply(cp, pres, pval);
            if(mEvals.empty() && pres == cp)
                break;
        }
#pragma omp critical
        {
            if(res == Math::invalidPos || mval < pval)
                mval = pval, res = pres;
        }
    }
    if(res == Math::invalidPos && expect)
        return expect();
    return res;
}
PointSampler& PointSampler::apply(const PointApplyor& func) {
    if(func)
        func(*this);
    return *this;
}
DiscreteSampler::DiscreteSampler(int siz) {
    for(int i = 0; i < siz; ++i)
        mPoints.push_back(i);
}
DiscreteSampler::DiscreteSampler(const std::vector<int>& points)
    : mPoints(points) {}
DiscreteSampler& DiscreteSampler::request(const DiscretePredFunc& req) {
    mReqs.push_back(req);
    return *this;
}
DiscreteSampler& DiscreteSampler::estimate(const DiscreteEvalFunc& eval) {
    mEvals.push_back(eval);
    return *this;
}
DiscreteSampler& DiscreteSampler::optional(const DiscretePredFunc& test) {
    mEvals.push_back(bool2val(test));
    return *this;
}
DiscreteSampler& DiscreteSampler::castPos(const Int2PointFunc& cast) {
    mCast = cast;
    return *this;
}
DiscreteSampler& DiscreteSampler::request(const PointPredFunc& req) {
    if(mCast) {
        Int2PointFunc cast = mCast;
        mReqs.push_back([cast, req](int id) { return req(cast(id)); });
    } else
        ERROR << "No cast!!!" << std::endl;
    return *this;
}
DiscreteSampler& DiscreteSampler::estimate(const PointEvalFunc& eval) {
    if(mCast) {
        Int2PointFunc cast = mCast;
        mEvals.push_back([cast, eval](int id) { return eval(cast(id)); });
    } else
        ERROR << "No cast!!!" << std::endl;
    return *this;
}
DiscreteSampler& DiscreteSampler::optional(const PointPredFunc& test) {
    if(mCast) {
        Int2PointFunc cast = mCast;
        PointEvalFunc func = bool2val(test);
        mEvals.push_back([cast, func](int id) { return func(cast(id)); });
    } else
        ERROR << "No cast!!!" << std::endl;
    return *this;
}
int DiscreteSampler::sample() const {
    if(mPoints.empty())
        return -1;
    if(mPoints.size() == 1)
        return mPoints.front();
    int res = -1;
    FixedSizeArray mval;
    for(int i : mPoints) {
        bool skip = false;
        for(const auto& p : mReqs)
            if(!p(i).test()) {
                skip = true;
                break;
            }
        if(skip)
            continue;
        if(mEvals.empty())
            return i;
        FixedSizeArray cval;
        for(const auto& p : mEvals) {
            FT val = p(i).get();
            if(val == Math::nan) {
                skip = true;
                break;
            }
            cval.pushBack(val);
            if(cval < mval)
                break;
        }
        if(skip)
            continue;
        if(res == -1 || mval < cval)
            mval = cval, res = i;
    }
    return res;
}
std::vector<int> DiscreteSampler::sample(size_t cnt) const {
    using Value = std::pair<FixedSizeArray, int>;
    std::priority_queue<Value, std::vector<Value>, std::greater<Value>> heap;
    for(int i : mPoints) {
        bool skip = false;
        for(const auto& p : mReqs)
            if(!p(i).test()) {
                skip = true;
                break;
            }
        if(skip)
            continue;
        FixedSizeArray cval;
        for(const auto& p : mEvals) {
            FT val = p(i).get();
            if(val == Math::nan) {
                skip = true;
                break;
            }
            cval.pushBack(val);
        }
        if(skip)
            continue;
        heap.emplace(cval, i);
        if(mEvals.empty() && heap.size() == cnt)
            break;
        while(heap.size() > cnt)
            heap.pop();
    }
    std::vector<int> res;
    while(heap.size()) {
        Value v = heap.top();
        heap.pop();
        res.push_back(v.second);
    }
    std::reverse(res.begin(), res.end());
    return res;
}
DiscreteSampler& DiscreteSampler::apply(const DiscreteApplyor& func) {
    if(func)
        func(*this);
    return *this;
}
Point DiscreteSampler::castToPos(int id) const {
    return mCast(id);
}

static uint32_t index = 0;
FT uniformD1() {
    return Math::halton3(index) * 2.0 - 1.0;
}
FT uniformD2() {
    return Math::halton5(index) * 2.0 - 1.0;
}

PointSampler circleSampler(const Point& base, FT rad) {
    FT r2 = rad * rad;
    return PointSampler([=](int, int) {
               while(true) {
                   ++index;
                   FT cx = uniformD1(), cy = uniformD2();
                   if(cx * cx + cy * cy < r2)
                       return Point(base.x + cx, base.y + cy);
               }
           })
        .hotRequest(distance(base) < rad);
}
PointSampler moveSampler(const Point& base, FT rad) {
    FT r2 = rad * rad;
    return PointSampler([=](int, int) {
               while(true) {
                   ++index;
                   FT cx = uniformD1(), cy = uniformD2();
                   cx = copysign(sqrt(fabs(cx)) * rad, cx),
                   cy = copysign(sqrt(fabs(cy)) * rad, cy);
                   Point off(cx, cy);
                   if(cx * cx + cy * cy < r2) {
                       if(length(off) / rad > 0.99)
                           off = Math::scale(off, rad);
                       return base + off;
                   }
               }
           })
        .hotRequest(distance(base) < rad);
}
PointSampler stableSampler(const Point& base, FT rad) {
    FT r2 = rad * rad;
    return PointSampler([=](int, int) {
               while(true) {
                   ++index;
                   FT cx = uniformD1(), cy = uniformD2();
                   cx = copysign(cx * cx * rad, cx),
                   cy = copysign(cy * cy * rad, cy);
                   if(cx * cx + cy * cy < r2)
                       return Point(base.x + cx, base.y + cy);
               }
           })
        .hotRequest(distance(base) < rad);
}

PointEvalFunc distance(const Point& base) {
    return [=](const Point& p) { return dist(p, base); };
}
PointEvalFunc dot(const Point& base, const Point& dst) {
    Point dir = dst - base;
    return [=](const Point& p) { return Math::dot(p - base, dir); };
}
PointEvalFunc angle(const Point& base, const Point& dst) {
    Point dir = dst - base;
    return [=](const Point& p) {
        Point off = p - base;
        return Math::angle(off, dir);
    };
}
PointEvalFunc distanceCloseTo(const Point& base, FT rad) {
    return [=](const Point& p) { return fabs(dist(p, base) - rad); };
}
Int2PointFunc myUnitPos() {
    return [](int id) { return getMyUnitPos(id); };
}
Int2PointFunc allUnitPos() {
    return [](int id) { return getUnit(id).position; };
}
Int2PointFunc estiUnitPos() {
    return [](int id) { return estimate(id) + getUnit(id).position; };
}
Int2PointFunc bonusPos() {
    return [](int id) { return getMap().bonus_places[id]; };
}
Int2PointFunc crystalPos() {
    return [](int id) { return getCrystal(id).position; };
}
Int2PointFunc crystalInitPos() {
    return [](int id) { return getMap().crystal_places[id]; };
}
DiscretePredFunc isSelf() {
    return [](int id) { return getFac(id) == getMyFac(); };
}
DiscretePredFunc isEnemy() {
    return [](int id) { return getFac(id) != getMyFac(); };
}
DiscretePredFunc isNotEq(int id) {
    return [id](int i) { return id != i; };
}
DiscretePredFunc isEnemyFac() {
    return [](int id) { return id != getMyFac(); };
}
DiscretePredFunc isAlive() {
    return [](int id) { return getUnit(id).hp > 0; };
}
DiscreteEvalFunc unitHP() {
    return [](int id) { return getUnit(id).hp; };
}
Int2PointFunc ghostPos(const Point& base) {
    return [=](int id) {
        if(getUnit(id).hp > 0)
            return getUnit(id).position;
        Point bp = getMap().birth_places[getFac(id)][getNum(id)];
        FT rdis =
            dist(bp, base) + getUnit(id).death_time * CONST::human_velocity;
        return base + Math::scale(bp - base, rdis);
    };
}
