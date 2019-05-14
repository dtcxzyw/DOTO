#include "Common.hpp"
std::ostringstream& debug() {
    static std::ostringstream ss;
    return ss;
}
std::ostream& error(const SourceLocation& loc) {
    return debug() << "#ERROR# [" << loc.file << ":"
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
    FT val = 1e20;
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
