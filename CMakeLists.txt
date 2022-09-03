# Sparkfun SparkFun_MMA8452Q Library
# https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
# Beerware License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "src/SparkFun_MMA8452Q.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES arduino)

project(SparkFun_MMA8452Q)
