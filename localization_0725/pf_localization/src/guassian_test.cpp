#include <iostream>
#include <random>

int main()
{

    std::default_random_engine engine;
    std::normal_distribution<double> guass(0., 1.);
    for (int i = 0; i < 10; i++)
    {
        std::cout << guass(engine) << std::endl;
    }

    std::cout << "<<<<<" << std::endl;

    for (int i = 0; i < 10; i++)
    {
        guass = std::normal_distribution<double>(2., 1.);
        std::cout << guass(engine) << std::endl;
    }
    return 0;
}