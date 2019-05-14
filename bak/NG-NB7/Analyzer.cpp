#include "Analyzer.hpp"
#include "Common.hpp"
#include "Math.hpp"
#include <algorithm>
#include <cmath>
#include <map>
const Point invalidPos = Point(-1, -1);
std::vector<Fireball> newFireBall() {
    static std::vector<Fireball> ofbs;
    std::vector<Fireball> res;
    for(const Fireball& nfb : getLogic().fireballs) {
        if(std::find_if(
               ofbs.begin(), ofbs.end(),
               [&](const Fireball& ofb) {
                   return ofb.from_number ==
                       nfb.from_number &&
                       fabs(nfb.rotation -
                            ofb.rotation) <
                       Math::bias &&
                       dist(nfb.position,
                            ofb.position) <
                       CONST::fireball_velocity +
                           Math::hurtBias;
               }) == ofbs.end())
            res.push_back(nfb);
    }
    ofbs = getLogic().fireballs;
    return res;
}
std::vector<Meteor> newMeteor() {
    static std::vector<Meteor> omes;
    std::vector<Meteor> res;
    for(const Meteor& nme : getLogic().meteors) {
        if(std::find_if(omes.begin(), omes.end(),
                        [&](const Meteor& ome) {
                            return ome.from_number ==
                                nme.from_number &&
                                dist(nme.position,
                                     ome.position) <
                                Math::bias;
                        }) == omes.end())
            res.push_back(nme);
    }
    omes = getLogic().meteors;
    return res;
}
void analyseUsage(int fac, int nfb, int nme) {
    // alive
    static std::map<int, int> frames;
    for(int i = 0; i < humanSiz(); ++i) {
        if(getUnit(getID(fac, i)).hp > 0)
            ++frames[fac];
    }
    info() << "alive ratio:"
           << 100.0 * frames[fac] /
            (getLogic().frame * humanSiz())
           << "%" << std::endl;
    // move
    static std::map<int, FT> move;
    static std::map<int, Point> pos;
    for(int i = 0; i < humanSiz(); ++i) {
        int id = getID(fac, i);
        if(getUnit(id).hp > 0) {
            Point cp = getUnit(id).position;
            if(pos.count(id))
                move[fac] += dist(cp, pos[id]);
            pos[id] = cp;
        } else
            pos.erase(id);
    }
    FT totFlash =
        frames[fac] / CONST::human_flash_interval;
    FT totMove = (frames[fac] - totFlash) *
            CONST::human_velocity +
        totFlash * CONST::flash_distance;
    info() << "move ratio:"
           << 100.0 * move[fac] / totMove << "%"
           << std::endl;
    // fire
    static std::map<int, int> fireCnt;
    fireCnt[fac] += nfb;
    int totFire =
        std::max(1, frames[fac] /
                     CONST::human_fireball_interval);
    info() << "fire ratio:"
           << 100.0 * fireCnt[fac] / totFire << "%"
           << "(" << fireCnt[fac] << "/" << totFire
           << ")" << std::endl;
    // meteor
    static std::map<int, int> meteorCnt;
    meteorCnt[fac] += nme;
    int totMeteor = std::max(
        1, frames[fac] / CONST::human_meteor_interval);
    info() << "meteor ratio:"
           << 100.0 * meteorCnt[fac] / totMeteor << "%"
           << "(" << meteorCnt[fac] << "/" << totMeteor
           << ")" << std::endl;
}
struct Factor final {
    FT fhp, fcdis, fhdis;
    Factor(FT fhp, FT fcdis, FT fhdis)
        : fhp(fhp), fcdis(fcdis), fhdis(fhdis) {}
    Factor() : Factor(0.0, 0.0, 0.0) {}
    FT eval(FT hp, FT dis, FT cdis, FT hdis) const {
        return fhp * hp + dis + fcdis * cdis +
            fhdis * hdis;
    }
    Factor shuffle(FT rad) const {
        auto gen = [rad] {
            return Math::uniform(-rad, rad);
        };
        return Factor(fhp + gen(), fcdis + gen(),
                      fhdis + gen());
    }
    void print(FT rat) const {
        info() << "eval factor: hp=" << fhp
               << " dis=1.0 cdis=" << fcdis
               << " hdis=" << fhdis
               << " ratio=" << 100.0 * rat << "%"
               << std::endl;
    }
};
struct Frame final {
    std::vector<std::pair<Point, int>> ems;
    std::vector<std::pair<Point, int>> fbs;
    Point helpPos, cryPos;
    bool help;
    std::pair<int, int>
    sample(const Factor& fac) const {
        int cnt = 0, hsiz = ems.size();
        for(const std::pair<Point, int>& fb : fbs) {
            int obj = -1;
            FT mval = 1e20;
            Point cp = fb.first;
            for(int i = 0; i < hsiz; ++i) {
                Point op = ems[i].first;
                FT cval = fac.eval(
                    ems[i].second, dist(cp, op),
                    dist(op, cryPos),
                    help ? dist(op, helpPos) : 0.0);
                if(cval < mval)
                    mval = cval, obj = i;
            }
            if(obj == fb.second)
                ++cnt;
        }
        return std::pair<int, int>(fbs.size(), cnt);
    }
};
void analyseEval(int fac,
                 const std::vector<Fireball>& nfb) {
    static std::map<int, Factor> facs;
    static std::map<int, std::vector<Frame>> frames;
    {
        Frame cfr;
        for(const Human& hu : getLogic().humans) {
            if(getFac(hu.number) == fac || hu.hp < 0)
                continue;
            cfr.ems.emplace_back(hu.position, hu.hp);
        }
        int hsiz = cfr.ems.size();
        for(const Fireball& fb : nfb) {
            const Human& hu = getUnit(fb.from_number);
            if(hu.hp < 0)
                continue;
            Point cp = hu.position;
            Point sdir(cos(fb.rotation),
                       sin(fb.rotation));
            int obj = -1;
            FT mang = 1e20;
            for(int i = 0; i < hsiz; ++i) {
                Point op = cfr.ems[i].first;
                Point dir = op - cp;
                FT cang = fabs(
                    atan2(Math::dot(sdir, dir),
                          Math::cross(sdir, dir)));
                if(isfinite(cang) && cang < mang)
                    obj = i, mang = cang;
            }
            if(obj != -1)
                cfr.fbs.emplace_back(cp, obj);
        }
        cfr.cryPos = getCrystal(fac).position;
        cfr.help = false;
        for(int i = 0; i < facSiz(); ++i) {
            const Crystal& cry = getCrystal(i);
            if(cry.belong != -1 &&
               getFac(cry.belong) == fac) {
                cfr.help = true;
                cfr.helpPos =
                    getUnit(cry.belong).position;
                break;
            }
        }
        if(cfr.fbs.size())
            frames[fac].push_back(cfr);
    }
    auto eval = [fac](const Factor& fa) {
        const std::vector<Frame>& frs = frames[fac];
        int A = 0, B = 0;
        for(const Frame& fr : frs) {
            std::pair<int, int> res = fr.sample(fa);
            A += res.first, B += res.second;
            if(A > 3000)
                break;
        }
        if(A == 0)
            return 1.0;
        return static_cast<FT>(B) / A;
    };
    Factor& mfa = facs[fac];
    FT mrat = eval(mfa);
    for(int i = 0; i < 10; ++i) {
        Factor cfa = mfa.shuffle(1.0 - mrat);
        FT crat = eval(cfa);
        if(crat > mrat)
            mrat = crat, mfa = cfa;
    }
    mfa.print(mrat);
}
template <typename T>
std::vector<T> filter(const std::vector<T>& A,
                      int fac) {
    std::vector<T> res;
    for(const T& a : A)
        if(getFac(a.from_number) == fac)
            res.push_back(a);
    return res;
}
void doAnalyse() {
    std::vector<Fireball> nfb = newFireBall();
    std::vector<Meteor> nme = newMeteor();
    for(int i = 0; i < facSiz(); ++i) {
        if(i == getMyFac())
            continue;
        info() << "analyse fac " << i << std::endl;
        std::vector<Fireball> infb = filter(nfb, i);
        std::vector<Meteor> inme = filter(nme, i);
        analyseUsage(i, infb.size(), inme.size());
        // analyseEval(i, infb);
    }
}
