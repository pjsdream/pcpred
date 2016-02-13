#include <pcpred/feature/feature.h>

using namespace pcpred;


Feature::Feature()
{
}

double Feature::kernel(const Feature &x)
{
    return 1.;
}

