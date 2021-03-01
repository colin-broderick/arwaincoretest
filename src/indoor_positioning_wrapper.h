#include <array>

class IndoorPositioningWrapper
{
    private:
        double m_x = 0;
        double m_y = 0;
        double m_z = 0;

        // TODO Create inner IPS object with Ross's rust library.

    public:
        void update(const double &time, const double &x, const double &y, const double &z);
        std::array<double, 3> getPosition();
        double getX();
        double getY();
        double getZ();
};
