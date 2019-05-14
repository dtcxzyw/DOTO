#include "Common.hpp"
#include "Math.hpp"
std::ostringstream& msg() {
    static std::ostringstream ss;
    return ss;
}
std::ostream& debug() {
    static std::ostringstream ss;
    ss.str("");
    return ss;
    // return msg();
}
std::ostream& info() {
    return msg();
}
std::ostream& error(const SourceLocation& loc) {
    return msg() << "#ERROR# [" << loc.file << ":"
                 << loc.line << "]";
}
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
FT minDist(const Point& pos, bool self) {
    FT val = Math::inf;
    for(int i = 0; i < facSiz(); ++i) {
        if(self ^ (i == getMyFac()))
            continue;
        for(int j = 0; j < humanSiz(); ++j) {
            if(getUnit(i, j).hp > 0) {
                val = std::min(
                    val,
                    dist(pos, getUnit(i, j).position));
            }
        }
    }
    return val;
}
bool hasMaxScore() {
    info() << "scores=[";
    for(FT sc : getLogic().scores)
        info() << sc << ",";
    info() << "]" << std::endl;
    FT msc = getLogic().scores[getMyFac()];
    for(int i = 0; i < facSiz(); ++i) {
        if(i == getMyFac())
            continue;
        if(getLogic().scores[i] > msc)
            return false;
    }
    return true;
}
