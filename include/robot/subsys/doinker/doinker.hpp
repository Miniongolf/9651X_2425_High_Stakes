#include "util.hpp"

class Doinker {
    public:
        enum Side { LEFT = -1, BOTH = 0, RIGHT = 1 };

        Doinker(PistonPtr leftPiston, PistonPtr rightPiston)
            : m_left(std::move(leftPiston)),
              m_right(std::move(rightPiston)) {};

        void extend(Side side) {
            if (side != LEFT) { m_right->extend(); }
            if (side != RIGHT) { m_left->extend(); }
        }

        void retract(Side side) {
            if (side != LEFT) { m_right->retract(); }
            if (side != RIGHT) { m_left->retract(); }
        }

        void toggle(Side side) {
            if (side != LEFT) { m_right->toggle(); }
            if (side != RIGHT) { m_left->toggle(); }
        }
    protected:
        PistonPtr m_left;
        PistonPtr m_right;
};