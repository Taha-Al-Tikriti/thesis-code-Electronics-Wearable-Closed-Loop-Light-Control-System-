#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>       // needed for the wifi stuff (recieving prescription + sending logs)
#include "esp_sleep.h"  // needed for the deep sleep + touch wakeup functions


// pin definitions, taken from the schematic (ESP32-S3-WROOM-1-N16R8)
// RGB leds
#define PIN_LED_RED         4
#define PIN_LED_GREEN       5
#define PIN_LED_BLUE        6

// status indicator leds (led 1, 2, 3)
#define PIN_IND_1           7
#define PIN_IND_2           15
#define PIN_IND_3           16

// body Temp. Sensor (MAX30205MTA+T)
#define PIN_TEMP_SDA        17
#define PIN_TEMP_SCL        18
#define PIN_TEMP_OS         8

// USB Interface
#define PIN_USB_DN          19
#define PIN_USB_DP          20

// Tristimulus light sensor (TCS34303M) - this is what reads the xyz colour
#define PIN_ALS_SDA         10
#define PIN_ALS_SCL         11
#define PIN_ALS_INT         12

// SpO2 / heart rate Sensor (MAX30101EFD+T)
#define PIN_SPO2_INT        13
#define PIN_SPO2_SCL        14
#define PIN_SPO2_SDA        21

// touchpad (wake up button basicaly)
#define PIN_TOUCH_WAKE      1

// pwm settings for the rgb leds
#define PWM_FREQ_HZ         5000
#define PWM_RES_BITS        10
#define PWM_MAX_DUTY        1023


// python exported H-inf lookup table start
// (dont touch this table, its generated from python, just copy pasted in)

#define NUM_LOOKUP_MODELS 5
#define K_STATES 15
#define K_INPUTS 3
#define K_OUTPUTS 3

const float A_GRID[NUM_LOOKUP_MODELS][K_STATES][K_STATES] = {
  {
    {9.999000e-01f, -0.000000e+00f, -1.057500e-16f, 2.643750e-17f, 3.965625e-17f, 6.609374e-18f, 6.609374e-18f, 6.609374e-18f, 8.261718e-19f, 3.172500e-16f, -4.230000e-16f, 1.321875e-17f, 1.057500e-16f, -0.000000e+00f, -3.304687e-18f},
    {0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {3.565536e+00f, -2.123373e+00f, -5.121597e-01f, -6.425938e-01f, 3.902120e-01f, -4.529024e-02f, -1.378480e-01f, 1.487808e-01f, 1.880307e-02f, 8.270961e-01f, -2.891865e+00f, -2.890380e-01f, 2.368661e+00f, 4.478018e-02f, -5.377809e-02f},
    {-2.181127e+00f, 1.623813e+00f, 2.931422e-01f, 3.902120e-01f, -3.018211e-01f, -3.838531e-02f, 1.496570e-01f, -2.256635e-03f, -1.829261e-02f, -2.890380e-01f, 2.368661e+00f, 5.755303e-01f, -8.065452e-01f, 4.528952e-02f, 9.885624e-02f},
    {9.547211e-02f, -8.189260e-02f, 2.361362e-01f, -4.529024e-02f, -3.838531e-02f, -1.759582e-02f, 6.416392e-03f, -2.230909e-03f, 9.473514e-02f, 4.478018e-02f, -5.377809e-02f, 4.528952e-02f, 9.885624e-02f, 3.543711e-01f, 7.095273e-01f},
    {-2.443388e-16f, 1.863823e-16f, 3.454743e-17f, -2.961132e-17f, -5.628845e-17f, -7.906055e-18f, 3.333333e-01f, 1.663973e-16f, -2.477263e-15f, 4.440892e-16f, 3.200000e+00f, 2.220446e-16f, 1.706667e+00f, 3.700743e-17f, 1.066667e+00f},
    {2.304340e-17f, -9.205096e-17f, -2.005699e-17f, -5.748274e-17f, -4.096222e-17f, 8.146532e-19f, -1.213612e-17f, 3.333333e-01f, -1.332853e-16f, 1.480297e-16f, 1.706667e+00f, 4.440892e-16f, 4.693333e+00f, 3.700743e-17f, 4.266667e-01f},
    {2.775558e-17f, -2.312965e-17f, -4.625929e-17f, 6.162976e-33f, 1.156482e-18f, -8.818178e-17f, 2.795796e-15f, 1.025366e-15f, 3.333333e-01f, -1.779379e-18f, -1.846778e-14f, 5.551115e-17f, 6.400000e-01f, 5.733261e-16f, 5.333333e+00f},
    {-5.348304e-01f, 3.185059e-01f, 7.682396e-02f, -3.610923e-03f, -5.853180e-02f, 6.793535e-03f, 2.067719e-02f, -2.231711e-02f, -2.820461e-03f, -6.240644e-01f, 1.433780e+00f, 4.335570e-02f, -3.552992e-01f, -6.717028e-03f, 8.066714e-03f},
    {2.674152e-01f, -1.592530e-01f, -3.841198e-02f, 1.805461e-03f, 2.926590e-02f, -3.396768e-03f, -1.033860e-02f, 1.115856e-02f, 1.410231e-03f, -1.879678e-01f, 2.831102e-01f, -2.167785e-02f, 1.776496e-01f, 3.358514e-03f, -4.033357e-03f},
    {3.271690e-01f, -2.435719e-01f, -4.397133e-02f, -5.853180e-02f, -5.472683e-02f, 5.757797e-03f, -2.244855e-02f, 3.384952e-04f, 2.743892e-03f, 4.335570e-02f, -3.552992e-01f, -5.863295e-01f, 1.120982e+00f, -6.793428e-03f, -1.482844e-02f},
    {-1.635845e-01f, 1.217860e-01f, 2.198567e-02f, 2.926590e-02f, 2.736342e-02f, -2.878899e-03f, 1.122428e-02f, -1.692476e-04f, -1.371946e-03f, -2.167785e-02f, 1.776496e-01f, -2.068352e-01f, 4.395091e-01f, 3.396714e-03f, 7.414218e-03f},
    {-1.432082e-02f, 1.228389e-02f, -3.542042e-02f, 6.793535e-03f, 5.757797e-03f, -9.736063e-02f, -9.624588e-04f, 3.346363e-04f, -1.421027e-02f, -6.717028e-03f, 8.066714e-03f, -6.793428e-03f, -1.482844e-02f, -5.531557e-01f, 8.935709e-01f},
    {7.160408e-03f, -6.141945e-03f, 1.771021e-02f, -3.396768e-03f, -2.878899e-03f, 4.868031e-02f, 4.812294e-04f, -1.673182e-04f, 7.105135e-03f, 3.358514e-03f, -4.033357e-03f, 3.396714e-03f, 7.414218e-03f, -2.234222e-01f, 5.532146e-01f},
  },
  {
    {9.999000e-01f, 5.984946e-16f, 7.481182e-17f, -0.000000e+00f, 3.740591e-17f, -3.117159e-18f, 3.117159e-17f, 1.246864e-17f, 2.337869e-18f, -4.987455e-16f, 3.989964e-16f, 1.496236e-16f, 9.974910e-17f, 6.234319e-18f, 3.117159e-18f},
    {0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {3.257850e+00f, -1.917264e+00f, -4.706252e-01f, -6.540416e-01f, 3.574824e-01f, -4.244278e-02f, -1.455754e-01f, 1.487028e-01f, 1.974695e-02f, 9.234427e-01f, -3.273182e+00f, -2.941985e-01f, 2.519488e+00f, 4.849779e-02f, -6.609400e-02f},
    {-1.971065e+00f, 1.504637e+00f, 2.620540e-01f, 3.574824e-01f, -3.419238e-01f, -3.659332e-02f, 1.497419e-01f, -1.000344e-02f, -1.878607e-02f, -2.941985e-01f, 2.519488e+00f, 6.676066e-01f, -1.055751e+00f, 5.047724e-02f, 9.180773e-02f},
    {8.874251e-02f, -8.025589e-02f, 2.536629e-01f, -4.244278e-02f, -3.659332e-02f, -8.063819e-02f, 5.300953e-03f, -2.038968e-03f, 8.716677e-02f, 4.849779e-02f, -6.609400e-02f, 5.047724e-02f, 9.180773e-02f, 4.397379e-01f, 5.652711e-01f},
    {-3.097256e-16f, 3.215408e-17f, -3.447170e-18f, -2.474295e-17f, -5.356651e-17f, -1.500690e-17f, 3.333333e-01f, -5.420522e-15f, -2.046097e-15f, 1.480297e-16f, 3.600000e+00f, 1.480297e-16f, 1.920000e+00f, 1.110223e-16f, 1.200000e+00f},
    {7.283679e-17f, -1.447708e-16f, -2.860458e-17f, -5.632991e-17f, -8.641636e-17f, 3.032180e-18f, 8.306473e-15f, 3.333333e-01f, 2.116601e-15f, 7.401487e-17f, 1.920000e+00f, 2.960595e-16f, 5.280000e+00f, 1.850372e-17f, 4.800000e-01f},
    {2.775558e-17f, -2.775558e-17f, -4.741578e-17f, -3.469447e-18f, -4.625929e-18f, -7.415943e-17f, 4.211330e-15f, 4.543421e-15f, 3.333333e-01f, -1.314909e-18f, -4.135780e-14f, 1.850372e-17f, 7.200000e-01f, 3.535945e-16f, 6.000000e+00f},
    {-4.886775e-01f, 2.875896e-01f, 7.059378e-02f, -1.893763e-03f, -5.362236e-02f, 6.366416e-03f, 2.183630e-02f, -2.230543e-02f, -2.962042e-03f, -6.385164e-01f, 1.490977e+00f, 4.412978e-02f, -3.779232e-01f, -7.274668e-03f, 9.914100e-03f},
    {2.443387e-01f, -1.437948e-01f, -3.529689e-02f, 9.468815e-04f, 2.681118e-02f, -3.183208e-03f, -1.091815e-02f, 1.115271e-02f, 1.481021e-03f, -1.807418e-01f, 2.545114e-01f, -2.206489e-02f, 1.889616e-01f, 3.637334e-03f, -4.957050e-03f},
    {2.956597e-01f, -2.256955e-01f, -3.930810e-02f, -5.362236e-02f, -4.871143e-02f, 5.488997e-03f, -2.246128e-02f, 1.500516e-03f, 2.817911e-03f, 4.412978e-02f, -3.779232e-01f, -6.001410e-01f, 1.158363e+00f, -7.571585e-03f, -1.377116e-02f},
    {-1.478299e-01f, 1.128477e-01f, 1.965405e-02f, 2.681118e-02f, 2.435572e-02f, -2.744499e-03f, 1.123064e-02f, -7.502582e-04f, -1.408956e-03f, -2.206489e-02f, 1.889616e-01f, -1.999295e-01f, 4.208186e-01f, 3.785793e-03f, 6.885580e-03f},
    {-1.331138e-02f, 1.203838e-02f, -3.804944e-02f, 6.366416e-03f, 5.488997e-03f, -8.790427e-02f, -7.951429e-04f, 3.058452e-04f, -1.307502e-02f, -7.274668e-03f, 9.914100e-03f, -7.571585e-03f, -1.377116e-02f, -5.659607e-01f, 9.152093e-01f},
    {6.655689e-03f, -6.019192e-03f, 1.902472e-02f, -3.183208e-03f, -2.744499e-03f, 4.395214e-02f, 3.975714e-04f, -1.529226e-04f, 6.537508e-03f, 3.637334e-03f, -4.957050e-03f, 3.785793e-03f, 6.885580e-03f, -2.170197e-01f, 5.423953e-01f},
  },
  {
    {9.999000e-01f, -0.000000e+00f, 9.672506e-17f, -7.738005e-17f, 3.869002e-17f, 2.418127e-18f, -1.934501e-17f, 9.672506e-18f, -2.418127e-18f, 2.321401e-16f, 9.285606e-16f, 1.160701e-16f, -3.095202e-16f, 1.209063e-17f, 3.869002e-17f},
    {0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {3.016157e+00f, -1.753061e+00f, -4.383046e-01f, -6.635337e-01f, 3.285272e-01f, -3.963154e-02f, -1.516060e-01f, 1.474718e-01f, 2.056280e-02f, 1.016596e+00f, -3.654433e+00f, -2.964826e-01f, 2.643877e+00f, 5.140830e-02f, -7.677088e-02f},
    {-1.803870e+00f, 1.413132e+00f, 2.369959e-01f, 3.285272e-01f, -3.767442e-01f, -3.456956e-02f, 1.486767e-01f, -1.711631e-02f, -1.895847e-02f, -2.964826e-01f, 2.643877e+00f, 7.589662e-01f, -1.328089e+00f, 5.467182e-02f, 8.521471e-02f},
    {8.357633e-02f, -7.957452e-02f, 2.712801e-01f, -3.963154e-02f, -3.456956e-02f, -1.360293e-01f, 4.305169e-03f, -1.699240e-03f, 7.937705e-02f, 5.140830e-02f, -7.677088e-02f, 5.467182e-02f, 8.521471e-02f, 5.269310e-01f, 3.799794e-01f},
    {-7.624708e-16f, 3.100150e-16f, -5.360825e-18f, 1.782216e-17f, -1.527570e-17f, -1.272122e-17f, 3.333333e-01f, -2.461406e-15f, -1.133078e-15f, 2.960595e-16f, 4.000000e+00f, 9.621933e-16f, 2.133333e+00f, 1.480297e-16f, 1.333333e+00f},
    {-1.966065e-16f, 8.471088e-17f, -4.031698e-18f, -7.674919e-18f, -3.264262e-17f, -4.628711e-18f, 2.382037e-15f, 3.333333e-01f, -8.454942e-16f, 7.401487e-17f, 2.133333e+00f, 5.921189e-16f, 5.866667e+00f, 3.700743e-17f, 5.333333e-01f},
    {3.700743e-17f, -2.103629e-30f, -4.394633e-17f, 1.156482e-18f, 1.156482e-18f, -9.728907e-17f, 2.391027e-15f, 2.453188e-16f, 3.333333e-01f, -1.512137e-18f, -1.668964e-14f, -5.094727e-30f, 8.000000e-01f, 6.458954e-16f, 6.666667e+00f},
    {-4.524236e-01f, 2.629591e-01f, 6.574570e-02f, -4.699493e-04f, -4.927907e-02f, 5.944732e-03f, 2.274090e-02f, -2.212077e-02f, -3.084419e-03f, -6.524895e-01f, 1.548165e+00f, 4.447238e-02f, -3.965816e-01f, -7.711245e-03f, 1.151563e-02f},
    {2.262118e-01f, -1.314796e-01f, -3.287285e-02f, 2.349746e-04f, 2.463954e-02f, -2.972366e-03f, -1.137045e-02f, 1.106039e-02f, 1.542210e-03f, -1.737553e-01f, 2.259175e-01f, -2.223619e-02f, 1.982908e-01f, 3.855623e-03f, -5.757816e-03f},
    {2.705805e-01f, -2.119698e-01f, -3.554939e-02f, -4.927907e-02f, -4.348838e-02f, 5.185435e-03f, -2.230151e-02f, 2.567447e-03f, 2.843771e-03f, 4.447238e-02f, -3.965816e-01f, -6.138449e-01f, 1.199213e+00f, -8.200773e-03f, -1.278221e-02f},
    {-1.352903e-01f, 1.059849e-01f, 1.777469e-02f, 2.463954e-02f, 2.174419e-02f, -2.592717e-03f, 1.115076e-02f, -1.283723e-03f, -1.421885e-03f, -2.223619e-02f, 1.982908e-01f, -1.930775e-01f, 4.003933e-01f, 4.100387e-03f, 6.391103e-03f},
    {-1.253645e-02f, 1.193618e-02f, -4.069201e-02f, 5.944732e-03f, 5.185435e-03f, -7.959560e-02f, -6.457753e-04f, 2.548860e-04f, -1.190656e-02f, -7.711245e-03f, 1.151563e-02f, -8.200773e-03f, -1.278221e-02f, -5.790396e-01f, 9.430031e-01f},
    {6.268224e-03f, -5.968089e-03f, 2.034601e-02f, -2.972366e-03f, -2.592717e-03f, 3.979780e-02f, 3.228877e-04f, -1.274430e-04f, 5.953279e-03f, 3.855623e-03f, -5.757816e-03f, 4.100387e-03f, 6.391103e-03f, -2.104802e-01f, 5.284985e-01f},
  },
  {
    {9.999000e-01f, 1.309152e-16f, -1.636440e-17f, -0.000000e+00f, -0.000000e+00f, 4.091100e-18f, -1.636440e-17f, -3.272880e-17f, 3.068325e-18f, 1.963728e-16f, 5.236608e-16f, 4.909320e-17f, -6.545760e-16f, -2.045550e-17f, -6.545760e-17f},
    {0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {2.820802e+00f, -1.618481e+00f, -4.124276e-01f, -6.714647e-01f, 3.027857e-01f, -3.691962e-02f, -1.562741e-01f, 1.454231e-01f, 2.126776e-02f, 1.106039e+00f, -4.032301e+00f, -2.962287e-01f, 2.743653e+00f, 5.360715e-02f, -8.578589e-02f},
    {-1.666963e+00f, 1.340874e+00f, 2.162262e-01f, 3.027857e-01f, -4.071762e-01f, -3.245122e-02f, 1.467927e-01f, -2.362135e-02f, -1.889385e-02f, -2.962287e-01f, 2.743653e+00f, 8.487998e-01f, -1.618627e+00f, 5.799081e-02f, 7.924974e-02f},
    {7.949700e-02f, -7.951246e-02f, 2.886279e-01f, -3.691962e-02f, -3.245122e-02f, -1.849502e-01f, 3.416417e-03f, -1.259580e-03f, 7.158456e-02f, 5.360715e-02f, -8.578589e-02f, 5.799081e-02f, 7.924974e-02f, 6.148412e-01f, 1.596887e-01f},
    {2.625191e-16f, 1.247790e-16f, 4.093436e-17f, -6.053150e-17f, -6.252931e-17f, -1.902262e-17f, 3.333333e-01f, -4.714185e-15f, 1.437069e-15f, 0.000000e+00f, 4.400000e+00f, -7.401487e-17f, 2.346667e+00f, 7.401487e-17f, 1.466667e+00f},
    {3.425304e-16f, -1.042747e-16f, -6.084224e-18f, -7.241643e-17f, -7.420619e-17f, -1.648285e-18f, 4.834868e-15f, 3.333333e-01f, -2.589801e-15f, 0.000000e+00f, 2.346667e+00f, 2.960595e-16f, 6.453333e+00f, 1.850372e-17f, 5.866667e-01f},
    {1.850372e-17f, -9.251859e-18f, -8.904914e-17f, 3.469447e-18f, 1.156482e-18f, -1.270685e-16f, -1.060494e-15f, 1.071799e-14f, 3.333333e-01f, -1.134666e-17f, -3.837946e-14f, 5.551115e-17f, 8.800000e-01f, 7.210667e-16f, 7.333333e+00f},
    {-4.231202e-01f, 2.427722e-01f, 6.186414e-02f, 7.197067e-04f, -4.541785e-02f, 5.537943e-03f, 2.344112e-02f, -2.181347e-02f, -3.190164e-03f, -6.659059e-01f, 1.604845e+00f, 4.443431e-02f, -4.115479e-01f, -8.041072e-03f, 1.286788e-02f},
    {2.115601e-01f, -1.213861e-01f, -3.093207e-02f, -3.598534e-04f, 2.270892e-02f, -2.768972e-03f, -1.172056e-02f, 1.090673e-02f, 1.595082e-03f, -1.670470e-01f, 1.975775e-01f, -2.221715e-02f, 2.057740e-01f, 4.020536e-03f, -6.433942e-03f},
    {2.500445e-01f, -2.011311e-01f, -3.243393e-02f, -4.541785e-02f, -3.892357e-02f, 4.867684e-03f, -2.201890e-02f, 3.543203e-03f, 2.834078e-03f, 4.443431e-02f, -4.115479e-01f, -6.273200e-01f, 1.242794e+00f, -8.698622e-03f, -1.188746e-02f},
    {-1.250223e-01f, 1.005656e-01f, 1.621696e-02f, 2.270892e-02f, 1.946178e-02f, -2.433842e-03f, 1.100945e-02f, -1.771601e-03f, -1.417039e-03f, -2.221715e-02f, 2.057740e-01f, -1.863400e-01f, 3.786029e-01f, 4.349311e-03f, 5.943730e-03f},
    {-1.192455e-02f, 1.192687e-02f, -4.329418e-02f, 5.537943e-03f, 4.867684e-03f, -7.225747e-02f, -5.124625e-04f, 1.889370e-04f, -1.073768e-02f, -8.041072e-03f, 1.286788e-02f, -8.698622e-03f, -1.188746e-02f, -5.922262e-01f, 9.760467e-01f},
    {5.962275e-03f, -5.963435e-03f, 2.164709e-02f, -2.768972e-03f, -2.433842e-03f, 3.612874e-02f, 2.562313e-04f, -9.446852e-05f, 5.368842e-03f, 4.020536e-03f, -6.433942e-03f, 4.349311e-03f, 5.943730e-03f, -2.038869e-01f, 5.119767e-01f},
  },
  {
    {9.999000e-01f, -2.629820e-16f, 4.930913e-17f, 3.287275e-17f, -6.574550e-17f, -4.109094e-18f, 2.465456e-17f, 8.218188e-18f, -0.000000e+00f, -3.944730e-16f, -1.577892e-15f, 1.808001e-16f, 5.259640e-16f, 3.698185e-17f, 3.698185e-17f},
    {0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999000e-01f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f, 0.000000e+00f},
    {2.659039e+00f, -1.505585e+00f, -3.911945e-01f, -6.781315e-01f, 2.798000e-01f, -3.434088e-02f, -1.598482e-01f, 1.428053e-01f, 2.187674e-02f, 1.191538e+00f, -4.404622e+00f, -2.937974e-01f, 2.820966e+00f, 5.518959e-02f, -9.320782e-02f},
    {-1.552212e+00f, 1.282380e+00f, 1.986228e-01f, 2.798000e-01f, -4.339232e-01f, -3.032383e-02f, 1.443352e-01f, -2.955847e-02f, -1.865629e-02f, -2.937974e-01f, 2.820966e+00f, 9.365626e-01f, -1.923313e+00f, 6.054986e-02f, 7.397268e-02f},
    {7.619488e-02f, -7.984499e-02f, 3.054342e-01f, -3.434088e-02f, -3.032383e-02f, -2.283553e-01f, 2.622116e-03f, -7.554353e-04f, 6.393155e-02f, 5.518959e-02f, -9.320782e-02f, 6.054986e-02f, 7.397268e-02f, 7.026108e-01f, -9.015124e-02f},
    {-5.556620e-16f, 5.939632e-16f, 1.233984e-17f, -8.771624e-17f, -1.389988e-16f, -3.135472e-18f, 3.333333e-01f, 1.545207e-16f, 2.283155e-15f, 1.480297e-16f, 4.800000e+00f, -7.401487e-17f, 2.560000e+00f, 7.401487e-17f, 1.600000e+00f},
    {1.576761e-17f, 7.135564e-17f, -4.107934e-17f, -1.062264e-16f, -1.155538e-16f, 5.770994e-18f, 1.884320e-15f, 3.333333e-01f, 1.183830e-14f, 1.480297e-16f, 2.560000e+00f, 2.960595e-16f, 7.040000e+00f, 1.850372e-17f, 6.400000e-01f},
    {3.700743e-17f, -2.775558e-17f, -7.401487e-17f, 1.156482e-18f, 1.156482e-18f, -5.478835e-17f, -1.233967e-15f, -1.733957e-14f, 3.333333e-01f, -3.635244e-18f, 8.690073e-14f, 3.700743e-17f, 9.600000e-01f, 3.587986e-16f, 8.000000e+00f},
    {-3.988559e-01f, 2.258377e-01f, 5.867917e-02f, 1.719727e-03f, -4.197000e-02f, 5.151132e-03f, 2.397723e-02f, -2.142079e-02f, -3.281512e-03f, -6.787306e-01f, 1.660693e+00f, 4.406961e-02f, -4.231449e-01f, -8.278439e-03f, 1.398117e-02f},
    {1.994279e-01f, -1.129189e-01f, -2.933959e-02f, -8.598636e-04f, 2.098500e-02f, -2.575566e-03f, -1.198861e-02f, 1.071040e-02f, 1.640756e-03f, -1.606347e-01f, 1.696533e-01f, -2.203480e-02f, 2.115725e-01f, 4.139219e-03f, -6.990587e-03f},
    {2.328318e-01f, -1.923570e-01f, -2.979343e-02f, -4.197000e-02f, -3.491153e-02f, 4.548575e-03f, -2.165029e-02f, 4.433771e-03f, 2.798443e-03f, 4.406961e-02f, -4.231449e-01f, -6.404844e-01f, 1.288497e+00f, -9.082478e-03f, -1.109590e-02f},
    {-1.164159e-01f, 9.617850e-02f, 1.489671e-02f, 2.098500e-02f, 1.745576e-02f, -2.274287e-03f, 1.082514e-02f, -2.216886e-03f, -1.399222e-03f, -2.203480e-02f, 2.115725e-01f, -1.797578e-01f, 3.557515e-01f, 4.541239e-03f, 5.547951e-03f},
    {-1.142923e-02f, 1.197675e-02f, -4.581513e-02f, 5.151132e-03f, 4.548575e-03f, -6.574671e-02f, -3.933174e-04f, 1.133153e-04f, -9.589733e-03f, -8.278439e-03f, 1.398117e-02f, -9.082478e-03f, -1.109590e-02f, -6.053916e-01f, 1.013523e+00f},
    {5.714616e-03f, -5.988374e-03f, 2.290756e-02f, -2.575566e-03f, -2.274287e-03f, 3.287335e-02f, 1.966587e-04f, -5.665765e-05f, 4.794867e-03f, 4.139219e-03f, -6.990587e-03f, 4.541239e-03f, 5.547951e-03f, -1.973042e-01f, 4.932387e-01f},
  },
};

const float B_GRID[NUM_LOOKUP_MODELS][K_STATES][K_INPUTS] = {
  {
    {9.999500e-02f, -0.000000e+00f, 3.304687e-18f},
    {0.000000e+00f, 9.999500e-02f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999500e-02f},
    {1.782768e-01f, -1.061686e-01f, -2.560799e-02f},
    {-1.090563e-01f, 8.119064e-02f, 1.465711e-02f},
    {4.773605e-03f, -4.094630e-03f, 1.180681e-02f},
    {-2.846742e-18f, 1.072629e-20f, -1.581849e-16f},
    {-1.980491e-17f, -4.263249e-17f, -6.168644e-17f},
    {7.459311e-17f, -3.469447e-17f, -3.751340e-17f},
    {-2.674152e-02f, 1.592530e-02f, 3.841198e-03f},
    {1.337076e-02f, -7.962648e-03f, -1.920599e-03f},
    {1.635845e-02f, -1.217860e-02f, -2.198567e-03f},
    {-8.179225e-03f, 6.089298e-03f, 1.099283e-03f},
    {-7.160408e-04f, 6.141945e-04f, -1.771021e-03f},
    {3.580204e-04f, -3.070973e-04f, 8.855106e-04f},
  },
  {
    {9.999500e-02f, -0.000000e+00f, 9.351478e-18f},
    {0.000000e+00f, 9.999500e-02f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999500e-02f},
    {1.628925e-01f, -9.586319e-02f, -2.353126e-02f},
    {-9.855324e-02f, 7.523183e-02f, 1.310270e-02f},
    {4.437126e-03f, -4.012795e-03f, 1.268315e-02f},
    {-4.403667e-17f, -3.754659e-16f, -2.167574e-16f},
    {1.203271e-16f, 1.780510e-16f, 7.280395e-17f},
    {2.312965e-18f, 1.679791e-16f, 1.713039e-16f},
    {-2.443387e-02f, 1.437948e-02f, 3.529689e-03f},
    {1.221694e-02f, -7.189739e-03f, -1.764845e-03f},
    {1.478299e-02f, -1.128477e-02f, -1.965405e-03f},
    {-7.391493e-03f, 5.642387e-03f, 9.827025e-04f},
    {-6.655689e-04f, 6.019192e-04f, -1.902472e-03f},
    {3.327844e-04f, -3.009596e-04f, 9.512360e-04f},
  },
  {
    {9.999500e-02f, -0.000000e+00f, -4.836253e-18f},
    {0.000000e+00f, 9.999500e-02f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999500e-02f},
    {1.508079e-01f, -8.765304e-02f, -2.191523e-02f},
    {-9.019350e-02f, 7.065660e-02f, 1.184980e-02f},
    {4.178816e-03f, -3.978726e-03f, 1.356400e-02f},
    {-1.497796e-16f, -1.767580e-16f, -1.127697e-16f},
    {-9.647457e-18f, 1.525946e-16f, 2.279066e-16f},
    {2.081668e-17f, 2.737972e-16f, 1.841698e-16f},
    {-2.262118e-02f, 1.314796e-02f, 3.287285e-03f},
    {1.131059e-02f, -6.573978e-03f, -1.643642e-03f},
    {1.352903e-02f, -1.059849e-02f, -1.777469e-03f},
    {-6.764513e-03f, 5.299245e-03f, 8.887346e-04f},
    {-6.268224e-04f, 5.968089e-04f, -2.034601e-03f},
    {3.134112e-04f, -2.984044e-04f, 1.017300e-03f},
  },
  {
    {9.999500e-02f, -2.454660e-17f, 4.091100e-18f},
    {0.000000e+00f, 9.999500e-02f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999500e-02f},
    {1.410401e-01f, -8.092406e-02f, -2.062138e-02f},
    {-8.334817e-02f, 6.704370e-02f, 1.081131e-02f},
    {3.974850e-03f, -3.975623e-03f, 1.443139e-02f},
    {-1.137626e-19f, -3.736398e-16f, -8.454199e-17f},
    {-1.797080e-17f, -8.841793e-17f, -8.057373e-16f},
    {-1.659552e-16f, -3.833739e-16f, -1.560528e-16f},
    {-2.115601e-02f, 1.213861e-02f, 3.093207e-03f},
    {1.057801e-02f, -6.069305e-03f, -1.546603e-03f},
    {1.250223e-02f, -1.005656e-02f, -1.621696e-03f},
    {-6.251113e-03f, 5.028278e-03f, 8.108481e-04f},
    {-5.962275e-04f, 5.963435e-04f, -2.164709e-03f},
    {2.981138e-04f, -2.981717e-04f, 1.082355e-03f},
  },
  {
    {9.999500e-02f, 2.465456e-17f, 2.054547e-18f},
    {0.000000e+00f, 9.999500e-02f, 0.000000e+00f},
    {0.000000e+00f, 0.000000e+00f, 9.999500e-02f},
    {1.329520e-01f, -7.527924e-02f, -1.955972e-02f},
    {-7.761059e-02f, 6.411900e-02f, 9.931142e-03f},
    {3.809744e-03f, -3.992250e-03f, 1.527171e-02f},
    {-1.078252e-16f, -1.413034e-16f, 7.866541e-17f},
    {-5.242489e-17f, -1.400112e-16f, 7.118525e-16f},
    {-1.908196e-17f, -2.286944e-16f, -3.960952e-17f},
    {-1.994279e-02f, 1.129189e-02f, 2.933959e-03f},
    {9.971397e-03f, -5.645943e-03f, -1.466979e-03f},
    {1.164159e-02f, -9.617850e-03f, -1.489671e-03f},
    {-5.820795e-03f, 4.808925e-03f, 7.448357e-04f},
    {-5.714616e-04f, 5.988374e-04f, -2.290756e-03f},
    {2.857308e-04f, -2.994187e-04f, 1.145378e-03f},
  },
};

const float C_GRID[NUM_LOOKUP_MODELS][K_OUTPUTS][K_STATES] = {
  {
    {2.139322e+01f, -1.274024e+01f, -3.072958e+00f, 1.444369e-01f, 2.341272e+00f, -2.717414e-01f, -8.270877e-01f, 8.926846e-01f, 1.128184e-01f, 4.962577e+00f, -1.735119e+01f, -1.734228e+00f, 1.421197e+01f, 2.686811e-01f, -3.226686e-01f},
    {-1.308676e+01f, 9.742877e+00f, 1.758853e+00f, 2.341272e+00f, 2.189073e+00f, -2.303119e-01f, 8.979422e-01f, -1.353981e-02f, -1.097557e-01f, -1.734228e+00f, 1.421197e+01f, 3.453182e+00f, -4.839271e+00f, 2.717371e-01f, 5.931375e-01f},
    {5.728327e-01f, -4.913556e-01f, 1.416817e+00f, -2.717414e-01f, -2.303119e-01f, 3.894425e+00f, 3.849835e-02f, -1.338545e-02f, 5.684108e-01f, 2.686811e-01f, -3.226686e-01f, 2.717371e-01f, 5.931375e-01f, 2.126226e+00f, 4.257164e+00f},
  },
  {
    {1.954710e+01f, -1.150358e+01f, -2.823751e+00f, 7.575052e-02f, 2.144894e+00f, -2.546567e-01f, -8.734521e-01f, 8.922171e-01f, 1.184817e-01f, 5.540656e+00f, -1.963909e+01f, -1.765191e+00f, 1.511693e+01f, 2.909867e-01f, -3.965640e-01f},
    {-1.182639e+01f, 9.027819e+00f, 1.572324e+00f, 2.144894e+00f, 1.948457e+00f, -2.195599e-01f, 8.984512e-01f, -6.002066e-02f, -1.127164e-01f, -1.765191e+00f, 1.511693e+01f, 4.005639e+00f, -6.334509e+00f, 3.028634e-01f, 5.508464e-01f},
    {5.324551e-01f, -4.815354e-01f, 1.521978e+00f, -2.546567e-01f, -2.195599e-01f, 3.516171e+00f, 3.180572e-02f, -1.223381e-02f, 5.230006e-01f, 2.909867e-01f, -3.965640e-01f, 3.028634e-01f, 5.508464e-01f, 2.638427e+00f, 3.391627e+00f},
  },
  {
    {1.809694e+01f, -1.051837e+01f, -2.629828e+00f, 1.879797e-02f, 1.971163e+00f, -2.377893e-01f, -9.096359e-01f, 8.848309e-01f, 1.233768e-01f, 6.099578e+00f, -2.192660e+01f, -1.778895e+00f, 1.586326e+01f, 3.084498e-01f, -4.606253e-01f},
    {-1.082322e+01f, 8.478792e+00f, 1.421975e+00f, 1.971163e+00f, 1.739535e+00f, -2.074174e-01f, 8.920604e-01f, -1.026979e-01f, -1.137508e-01f, -1.778895e+00f, 1.586326e+01f, 4.553797e+00f, -7.968533e+00f, 3.280309e-01f, 5.112883e-01f},
    {5.014580e-01f, -4.774471e-01f, 1.627680e+00f, -2.377893e-01f, -2.074174e-01f, 3.183824e+00f, 2.583101e-02f, -1.019544e-02f, 4.762623e-01f, 3.084498e-01f, -4.606253e-01f, 3.280309e-01f, 5.112883e-01f, 3.161586e+00f, 2.279876e+00f},
  },
  {
    {1.692481e+01f, -9.710887e+00f, -2.474566e+00f, -2.878827e-02f, 1.816714e+00f, -2.215177e-01f, -9.376448e-01f, 8.725388e-01f, 1.276066e-01f, 6.636236e+00f, -2.419380e+01f, -1.777372e+00f, 1.646192e+01f, 3.216429e-01f, -5.147153e-01f},
    {-1.000178e+01f, 8.045245e+00f, 1.297357e+00f, 1.816714e+00f, 1.556943e+00f, -1.947073e-01f, 8.807560e-01f, -1.417281e-01f, -1.133631e-01f, -1.777372e+00f, 1.646192e+01f, 5.092799e+00f, -9.711764e+00f, 3.479449e-01f, 4.754984e-01f},
    {4.769820e-01f, -4.770748e-01f, 1.731767e+00f, -2.215177e-01f, -1.947073e-01f, 2.890299e+00f, 2.049850e-02f, -7.557482e-03f, 4.295074e-01f, 3.216429e-01f, -5.147153e-01f, 3.479449e-01f, 4.754984e-01f, 3.689047e+00f, 9.581323e-01f},
  },
  {
    {1.595423e+01f, -9.033509e+00f, -2.347167e+00f, -6.878909e-02f, 1.678800e+00f, -2.060453e-01f, -9.590891e-01f, 8.568316e-01f, 1.312605e-01f, 7.149226e+00f, -2.642773e+01f, -1.762784e+00f, 1.692580e+01f, 3.311375e-01f, -5.592469e-01f},
    {-9.313271e+00f, 7.694280e+00f, 1.191737e+00f, 1.678800e+00f, 1.396461e+00f, -1.819430e-01f, 8.660114e-01f, -1.773508e-01f, -1.119377e-01f, -1.762784e+00f, 1.692580e+01f, 5.619376e+00f, -1.153988e+01f, 3.632991e-01f, 4.438361e-01f},
    {4.571693e-01f, -4.790699e-01f, 1.832605e+00f, -2.060453e-01f, -1.819430e-01f, 2.629868e+00f, 1.573269e-02f, -4.532612e-03f, 3.835893e-01f, 3.311375e-01f, -5.592469e-01f, 3.632991e-01f, 4.438361e-01f, 4.215665e+00f, -5.409075e-01f},
  },
};

const float D_GRID[NUM_LOOKUP_MODELS][K_OUTPUTS][K_INPUTS] = {
  {
    {1.069661e+00f, -6.370119e-01f, -1.536479e-01f},
    {-6.543380e-01f, 4.871439e-01f, 8.794267e-02f},
    {2.864163e-02f, -2.456778e-02f, 7.084085e-02f},
  },
  {
    {9.773549e-01f, -5.751791e-01f, -1.411876e-01f},
    {-5.913194e-01f, 4.513910e-01f, 7.861620e-02f},
    {2.662275e-02f, -2.407677e-02f, 7.609888e-02f},
  },
  {
    {9.048472e-01f, -5.259183e-01f, -1.314914e-01f},
    {-5.411610e-01f, 4.239396e-01f, 7.109877e-02f},
    {2.507290e-02f, -2.387236e-02f, 8.138402e-02f},
  },
  {
    {8.462405e-01f, -4.855444e-01f, -1.237283e-01f},
    {-5.000890e-01f, 4.022622e-01f, 6.486785e-02f},
    {2.384910e-02f, -2.385374e-02f, 8.658837e-02f},
  },
  {
    {7.977117e-01f, -4.516755e-01f, -1.173583e-01f},
    {-4.656636e-01f, 3.847140e-01f, 5.958685e-02f},
    {2.285846e-02f, -2.395350e-02f, 9.163026e-02f},
  },
};

// python exported H-inf lookup table end

// active controller matricies, loaded at startup depending on wich model the NN picks
float A_active[K_STATES][K_STATES];
float B_active[K_STATES][K_INPUTS];
float C_active[K_OUTPUTS][K_STATES];
float D_active[K_OUTPUTS][K_INPUTS];

// controller state vector, reference colour, and the ambient noise we measure at boot
float x_state[K_STATES] = {0.0f};
float target_ref[3] = {0.2f, 0.2f, 0.2f}; // gets overwritten by the prescription colours later
float ambient_noise[3] = {0.0f, 0.0f, 0.0f}; // xyz baseline noise, measured with leds off

// this is just the diode temp measured at boot, not the body temp sensor!
float g_startup_temp_c = 25.0f;

// most recent light sensor reading (after removing ambient noise), saved here so logSample()
// can grab it without passing it around everywhere
float y_corrected_global[3] = {0.0f, 0.0f, 0.0f};

// ---------------------------------------------------------
// WIFI + SERVER SETTINGS

// just placeholders for now
const char* WIFI_SSID = "WIFI_NAME";
const char* WIFI_PASS = "WIFI_PASSWORD";

const char*    SERVER_IP         = "192.168.1.50"; // remote server we send the log to at the end
const uint16_t SERVER_PORT       = 5000;
const uint16_t PRESCRIPTION_PORT = 5001;             // WE listen on this port for the prescription

WiFiServer prescriptionServer(PRESCRIPTION_PORT);
bool g_wifi_server_started = false; // so we only call .begin() on the server once

// i2c addresses for the 3 sensors, used when we check they are all conected
#define ADDR_ALS   0x39   // TCS34303M
#define ADDR_TEMP  0x48   // MAX30205
#define ADDR_SPO2  0x57   // MAX30101

// the ALS and temp sensor get there own Wire / Wire1 bus, heart rate sensor gets a 3rd
// bus since its on totaly seperate pins
TwoWire Wire2 = TwoWire(2);

 
// -----------------------------------------------------------
// PRESCRIPTION DATA (recieved over wifi from the external server)
 
// each "step" is one colour (in CIE1931 xyz) plus how long to show it for, in ms
struct ColorStep {
  float x;
  float y;
  float z;
  unsigned long duration_ms;
};

#define MAX_COLOR_STEPS 30
ColorStep prescription[MAX_COLOR_STEPS];
int numColorSteps = 0; // how many steps we actually got, filled in by tryReceivePrescriptionOnce()


// ----------------------------------------------------------
// SESSION LOG DATA (sent to the server once, after the session is finished)

// one row per second: timestamp + body temp + heart rate + light reading + wich colour was on
struct LogEntry {
  unsigned long timestamp_ms;
  float body_temp;
  float heart_rate;
  float light_x, light_y, light_z;
  int color_index;
};

#define MAX_LOG_ENTRIES 3600 // shud be enough for like an hour long session (1 sample/sec)
LogEntry sessionLog[MAX_LOG_ENTRIES];
int logCount = 0;

// -------------------------------------------------
// TIMER 1 (overall 3 minute "wake window" - if we cant get ready in time we sleep)

#define TIMER1_DURATION_MS 180000UL // 3 minutes
unsigned long timer1_start = 0;

void timer1Init() {
  timer1_start = millis();
}

bool timer1Expired() {
  return (millis() - timer1_start) >= TIMER1_DURATION_MS;
}

// =-==-==-==-==-==-==-==-==-==-==-==-==-==-===-==-==-==-===-==-=
// TOUCHPAD (this doubles as the sleep-wake pin AND a "hold to restart/erase"
// button while the device is awake, using an interrupt so it works no matter
// what else the code is doing at the time)

#define TOUCH_THRESHOLD 40 // lower reading = pad is being touched, tweak this if its too sensitive/not enough

volatile bool touch_pressed = false;
volatile unsigned long touch_start_ms = 0;

// flags set from the touch logic, checked all over the main flow so we can bail
// out of whatever blocking thing we were doing and jump back to the start
volatile bool g_restart_requested = false;
volatile bool g_erase_requested   = false;

// this is the actual interrupt handler - keep it SHORT, no delays/serial/etc in here
void IRAM_ATTR onTouchPress() {
  if (!touch_pressed) {
    touch_pressed  = true;
    touch_start_ms = millis(); // timer2 start
  }
}

// the touch peripheral only interrupts us on the PRESS, so to catch the RELEASE
// (and measure how long it was held) we just poll touchRead() every so often.
// called from all over the place so it stays responsive even mid-blocking-loop
void serviceTouch() {
  if (!touch_pressed) return;

  if (touchRead(PIN_TOUCH_WAKE) > TOUCH_THRESHOLD) {
    // pad let go = released?
    touch_pressed = false;
    unsigned long held_for = millis() - touch_start_ms; // time_elapsed

    if (held_for > 5000) {
      // t > 5s -> erase_prescription -> start
      g_erase_requested   = true;
      g_restart_requested = true;
    } else if (held_for < 2000) {
      // t < 2s -> restart (t1_init)
      g_restart_requested = true;
    }
    // else: between 2 and 5 seconds -> do_nothing
  }
}

// little helper macro, drop this at the top of any loop iteration that could run
// for a while, so a touch-restart actually interupts it instead of waiting
#define CHECK_RESTART() do { serviceTouch(); if (g_restart_requested) return; } while (0)

// delay() that still lets us notice a touch release wile waiting
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    serviceTouch();
    if (g_restart_requested) return;
    delay(20);
  }
}

//--------------- ----------------- --------------- ----------
// SYSTEM INDICATOR CODES

enum SystemState {
  STATE_AWAKE            = 0b001, // 001
  STATE_PERIPHERAL_ERROR = 0b010, // 010
  STATE_NETWORK_ERROR    = 0b011, // 011
  STATE_CONTROL_OK       = 0b100, // 100
  STATE_CONTROL_NOT_OK   = 0b101  // 101
};

void setStatusLEDs(uint8_t binary_code) {
  // just pushes the 3 bits out to the 3 indicator leds, msb first
  digitalWrite(PIN_IND_1, (binary_code & 0b100) ? HIGH : LOW);
  digitalWrite(PIN_IND_2, (binary_code & 0b010) ? HIGH : LOW);
  digitalWrite(PIN_IND_3, (binary_code & 0b001) ? HIGH : LOW);
}

//--------- --------------------------------------------------
// SMALL NEURAL NET, JUST PICKS WICH CONTROLLER MODEL TO USE

// in:  [ambient x, ambient y, ambient z, startup temp]
// out: an index from 0 to NUM_LOOKUP_MODELS - 1
int runNeuralNetworkInference(float ambX, float ambY, float ambZ, float tempC) {
  const float W1[4][4] = {
    { 0.25f, -0.10f,  0.05f,  0.01f},
    {-0.08f,  0.30f, -0.02f,  0.02f},
    { 0.02f, -0.05f,  0.22f,  0.00f},
    { 0.10f,  0.10f,  0.10f,  0.05f}
  };
  const float b1[4] = {0.1f, 0.1f, 0.1f, 0.0f};
  const float W2[4] = {0.8f, 1.2f, 0.9f, 0.5f};

  float h1[4] = {0.0f};
  float vin[4] = {ambX, ambY, ambZ, tempC};

  // hidden layer, 4 neurons, relu
  for (int i = 0; i < 4; i++) {
    float sum = 0.0f;
    for (int j = 0; j < 4; j++) {
      sum += W1[i][j] * vin[j];
    }
    sum += b1[i];
    h1[i] = (sum > 0.0f) ? sum : 0.0f; // relu just clamps negative stuff to 0
  }

  // output score
  float score = 0.0f;
  for (int i = 0; i < 4; i++) {
    score += W2[i] * h1[i];
  }

  // turn the score into an index we can actually use, and clamp so it doesnt go out of bounds
  int selected_idx = (int)floor(2.0f * score);
  if (selected_idx < 0) selected_idx = 0;
  if (selected_idx >= NUM_LOOKUP_MODELS) selected_idx = NUM_LOOKUP_MODELS - 1;

  return selected_idx;
}

void loadActiveController(int model_idx) {
  // just copies the picked model out of the big lookup table into the "active" matricies
  memcpy(A_active, A_GRID[model_idx], sizeof(A_active));
  memcpy(B_active, B_GRID[model_idx], sizeof(B_active));
  memcpy(C_active, C_GRID[model_idx], sizeof(C_active));
  memcpy(D_active, D_GRID[model_idx], sizeof(D_active));
}

// -------------------------------------------------------------------------------------------
// LED CALIBRATION (led_cal node - ambient noise + startup temp + NN model pick)

// note: this used to also check the light sensor is conected and halt forever if not,
// that check got moved out into checkAllPeripherals() so the main flow can retry/timeout
// properly instead of just hanging
void runLedCalibration() {
  // measure baseline ambient noise with the leds off
  ambient_noise[0] = 0.02f; // TODO replace these with actual Wire reads
  ambient_noise[1] = 0.02f;
  ambient_noise[2] = 0.02f;

  // quick n dirty temp measurment using the red led as a diode (dont ask lol)
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH);
  delayMicroseconds(50); // short pulse so we dont heat the led up and mess up the reading

  uint16_t raw_adc = analogRead(PIN_LED_RED);
  digitalWrite(PIN_LED_RED, LOW);

  float v_forward = (raw_adc / 4095.0f) * 3.3f; // 12 bit adc -> volts
  float v_ref_25c = 2.00f;                      // what the led reads at 25C
  float temp_coeff = -0.002f;                   // roughly -2mV per degree C

  g_startup_temp_c = 25.0f + ((v_ref_25c - v_forward) / (-temp_coeff));

  // ask the tiny NN wich controller model fits this environment best
  int matched_model_idx = runNeuralNetworkInference(
      ambient_noise[0],
      ambient_noise[1],
      ambient_noise[2],
      g_startup_temp_c
  );

  loadActiveController(matched_model_idx);
}

// not 100% sure what "ready" is supposed to check exactly, going with a basic sanity
// check on the temp reading for now - if its wildly out of range something probably
// went wrong with the calibration pulse
bool isCalibrationReady() {
  return (g_startup_temp_c > 0.0f) && (g_startup_temp_c < 60.0f);
}

// .........................--.........................--........................--
// WIFI CONNECT (n_conn / success? - one attempt, doesnt block forever)

bool tryConnectWifiOnce() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long attempt_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - attempt_start < 3000) {
    serviceTouch();
    if (g_restart_requested) return false; // dont get stuck here if the user wants to restart
    delay(100);
  }

  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !g_wifi_server_started) {
    prescriptionServer.begin(); // start listening for the prescription now were online
    g_wifi_server_started = true;
  }

  return connected;
}

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// PERIPHERAL CHECK (check_i2c / ack? - all 3 sensors, reported thru the leds)

bool checkAllPeripherals() {
  bool ok = true;

  Wire.beginTransmission(ADDR_ALS);
  if (Wire.endTransmission() != 0) ok = false;

  Wire1.beginTransmission(ADDR_TEMP);
  if (Wire1.endTransmission() != 0) ok = false;

  Wire2.beginTransmission(ADDR_SPO2);
  if (Wire2.endTransmission() != 0) ok = false;

  return ok;
}

// ---- ---- ------ ------- ---- ------ ------- ------ ------- -------- -------- 
// RECIEVE PRESCRIPTION (p_read / found? - one non blocking check, doesnt wait)

// format is just plain text, comma seperated:
//   count,x1,y1,z1,dur1,x2,y2,z2,dur2, .... ,xN,yN,zN,durN
bool tryReceivePrescriptionOnce() {
  WiFiClient client = prescriptionServer.available();
  if (!client) return false; // nobodys sent us anything yet, thats fine, try again later

  String line = client.readStringUntil('\n');
  client.stop();

  int startPos = 0;
  int commaPos = line.indexOf(',');
  numColorSteps = line.substring(0, commaPos).toInt();
  startPos = commaPos + 1;

  for (int i = 0; i < numColorSteps && i < MAX_COLOR_STEPS; i++) {
    commaPos = line.indexOf(',', startPos);
    prescription[i].x = line.substring(startPos, commaPos).toFloat();
    startPos = commaPos + 1;

    commaPos = line.indexOf(',', startPos);
    prescription[i].y = line.substring(startPos, commaPos).toFloat();
    startPos = commaPos + 1;

    commaPos = line.indexOf(',', startPos);
    prescription[i].z = line.substring(startPos, commaPos).toFloat();
    startPos = commaPos + 1;

    commaPos = line.indexOf(',', startPos);
    if (commaPos == -1) commaPos = line.length(); // last value in the line has no comma after it
    prescription[i].duration_ms = (unsigned long) line.substring(startPos, commaPos).toInt();
    startPos = commaPos + 1;
  }

  return numColorSteps > 0;
}

// ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
// SENSOR READS (body temp + heart rate)

float readBodyTemp() {
  // TODO: actually read this over Wire1 from the MAX30205, just a placeholder rn
  return 36.5f;
}

float readHeartRate() {
  // TODO: actually read this over Wire2 from the MAX30101, just a placeholder rn
  return 72.0f;
}

// grabs one row of data (temp, heart rate, light, wich colour was active) and saves it
// (this is "pack_data" - we just buffer it here, the actual send happens once at the
// very end in sendLogData(), not every cycle)
void logSample(int colorIndex) {
  if (logCount >= MAX_LOG_ENTRIES) return; // ran out of room, just stop logging i guess

  LogEntry entry;
  entry.timestamp_ms = millis();
  entry.body_temp     = readBodyTemp();
  entry.heart_rate     = readHeartRate();
  entry.light_x = y_corrected_global[0];
  entry.light_y = y_corrected_global[1];
  entry.light_z = y_corrected_global[2];
  entry.color_index = colorIndex;

  sessionLog[logCount] = entry;
  logCount++;
}

// ...........   ...........   ...........   ...........   ...........   ...........   ...........
// CONTROL STEP (color_ctrl - drives the leds towards whatever the current
// prescription colour is, using the H-inf controller matricies)

void runControlStep(float target[3]) {
  // 1. read the light sensor (fake numbers for now, swap in real Wire reads later)
  float y_raw[3] = {0.12f, 0.11f, 0.11f};

  // 2. take away the ambient noise from earlier
  float y_corrected[3];
  for (int i = 0; i < 3; i++) {
    y_corrected[i] = y_raw[i] - ambient_noise[i];
    y_corrected_global[i] = y_corrected[i]; // save for the logger
  }

  // 3. error = what we want minus what we got
  float e_err[3];
  for (int i = 0; i < 3; i++) {
    e_err[i] = target[i] - y_corrected[i];
  }

  // 4. u = C*x + D*e , this is the actual controller output
  float u_out[K_OUTPUTS] = {0.0f};
  for (int i = 0; i < K_OUTPUTS; i++) {
    for (int j = 0; j < K_STATES; j++) {
      u_out[i] += C_active[i][j] * x_state[j];
    }
    for (int j = 0; j < K_INPUTS; j++) {
      u_out[i] += D_active[i][j] * e_err[j];
    }
  }

  // 5. update states for next loop, x = A*x + B*e
  float x_next[K_STATES] = {0.0f};
  for (int i = 0; i < K_STATES; i++) {
    for (int j = 0; j < K_STATES; j++) {
      x_next[i] += A_active[i][j] * x_state[j];
    }
    for (int j = 0; j < K_INPUTS; j++) {
      x_next[i] += B_active[i][j] * e_err[j];
    }
  }
  memcpy(x_state, x_next, sizeof(x_state));

  // 6. clamp between 0 and 1 so we dont overdrive the leds, then write it out
  for (int i = 0; i < 3; i++) {
    u_out[i] = constrain(u_out[i], 0.0f, 1.0f);
  }

  ledcWrite(PIN_LED_RED,   (uint32_t)(u_out[0] * PWM_MAX_DUTY));
  ledcWrite(PIN_LED_GREEN, (uint32_t)(u_out[1] * PWM_MAX_DUTY));
  ledcWrite(PIN_LED_BLUE,  (uint32_t)(u_out[2] * PWM_MAX_DUTY));
}

// ====-====-====-====-====-====-====-====-====-====-====-====-====-====-
// SESSION LOOP (vitals + color_ctrl + pack_data + loop_check + leds_off)

// t_session = however long it takes to play the whole prescription once through.
// runs the colour control (10hz) and the vitals sampling (1hz) side by side untill
// were done, then turns the leds off.
void runPrescriptionSession() {
  unsigned long t_session = 0;
  for (int i = 0; i < numColorSteps; i++) t_session += prescription[i].duration_ms;

  unsigned long session_start   = millis();
  unsigned long color_step_start = millis();
  unsigned long last_control      = 0;
  unsigned long last_log          = 0;
  int current_color = 0;

  while (millis() - session_start <= t_session) { // loop_check: while t <= t_session?
    CHECK_RESTART();

    // move on to the next colour once the current ones duration is up
    if (current_color < numColorSteps &&
        millis() - color_step_start >= prescription[current_color].duration_ms) {
      current_color++;
      color_step_start = millis();
    }
    if (current_color >= numColorSteps) break; // played the whole thing, were done early

    float target[3] = {
      prescription[current_color].x,
      prescription[current_color].y,
      prescription[current_color].z
    };

    if (millis() - last_control >= 100) { // color_ctrl, 10hz same as b4
      last_control = millis();
      runControlStep(target);
    }

    if (millis() - last_log >= 1000) {    // vitals + pack_data, once a second
      last_log = millis();
      logSample(current_color);
    }
  }

  // leds_off
  ledcWrite(PIN_LED_RED, 0);
  ledcWrite(PIN_LED_GREEN, 0);
  ledcWrite(PIN_LED_BLUE, 0);
}

// ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// SEND THE LOGGED DATA (upload_data - only happens ONCE, here, after the
// session ends. not every cycle like the diagram kinda suggests, we're doing
// it batched like discussed)

void sendLogData() {
  WiFiClient client;

  if (!client.connect(SERVER_IP, SERVER_PORT)) {
    setStatusLEDs(STATE_NETWORK_ERROR);
    return; // cant reach the server, data just stays lost for now :(
  }

  // one comma seperated line per sample:
  // timestamp,body_temp,heart_rate,light_x,light_y,light_z,color_index
  for (int i = 0; i < logCount; i++) {
    LogEntry e = sessionLog[i];
    client.print(e.timestamp_ms);  client.print(",");
    client.print(e.body_temp);     client.print(",");
    client.print(e.heart_rate);    client.print(",");
    client.print(e.light_x);       client.print(",");
    client.print(e.light_y);       client.print(",");
    client.print(e.light_z);       client.print(",");
    client.print(e.color_index);
    client.print("\n");
  }

  client.stop();
}

// |||||-|||||-|||||- |||||-|||||-|||||- |||||-|||||-|||||- |||||-|||||-|||||- |||||-|||||-|||||-
// SLEEP MODE (sleep1 / sleep2 / sleep3 - all the same thing really, just drawn
// 3 times on the diagram. real deep sleep, touching the pad wakes it back up
// again which counts as a full reboot, so execution starts over from setup())

void goToSleep() {
  // turn the status leds off before we sleep, not strictly needed but saves a tiny bit of power
  digitalWrite(PIN_IND_1, LOW);
  digitalWrite(PIN_IND_2, LOW);
  digitalWrite(PIN_IND_3, LOW);

  esp_sleep_enable_touchpad_wakeup();
  esp_deep_sleep_start(); // this doesnt return, chip resets when the pad gets touched again
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// MAIN FLOW (this is basically the whole activity diagram, start to sleep)

// if a touch-restart happens partway through, this function just returns early
// (see CHECK_RESTART) and loop() calls it again, wich is the same as jumping
// back to "start" without actually rebooting the chip
void runMainFlow() {
  setStatusLEDs(STATE_AWAKE); // start
  timer1Init();                 // t1_init: timer 1 set to 3 minutes

  // ---- stage 1: get online and wait for the prescription (p_read / n_conn) ----
  bool got_prescription = false;
  while (!got_prescription) {
    CHECK_RESTART();

    bool wifi_ok = tryConnectWifiOnce(); // n_conn -> success?
    if (!wifi_ok) {
      setStatusLEDs(STATE_NETWORK_ERROR); // net_err
    } else {
      got_prescription = tryReceivePrescriptionOnce(); // p_read -> found?
    }

    if (got_prescription) break; // -> check_i2c

    if (timer1Expired()) { // t1_check1 -> fin1
      goToSleep();         // sleep1
      return;
    }

    CHECK_RESTART();
    smartDelay(200); // dont hammer the wifi/server, breath a bit between tries
  }

  // ---- stage 2: check peripherals + run led calibration (check_i2c / led_cal / ready) ----
  bool calibration_ready = false;
  while (!calibration_ready) {
    CHECK_RESTART();

    bool peripherals_ok = checkAllPeripherals(); // check_i2c -> ack?
    if (!peripherals_ok) {
      setStatusLEDs(STATE_PERIPHERAL_ERROR); // periph_err
    } else {
      runLedCalibration();                        // led_cal
      calibration_ready = isCalibrationReady();    // ready?
      setStatusLEDs(calibration_ready ? STATE_CONTROL_OK : STATE_CONTROL_NOT_OK); // lc_ok / lc_nok
    }

    if (calibration_ready) break; // -> delay1

    if (timer1Expired()) { // t1_check2 -> fin2
      goToSleep();         // sleep2
      return;
    }

    CHECK_RESTART();
    smartDelay(200);
  }

  smartDelay(10000); // delay1: delay 10s
  CHECK_RESTART();
  // t1_reset - not really needed anymore since were past the timeout-sensitive part,
  // but resetting it anyway just to match the diagram / incase we add stuff here later
  timer1Init();

  // ---- stage 3: the actual session (vitals + color_ctrl + pack_data + loop_check) ----
  runPrescriptionSession();
  CHECK_RESTART();

  smartDelay(10000); // delay2
  CHECK_RESTART();

  sendLogData();      // upload_data (batched, once, at the end)

  goToSleep();         // sleep3
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// SETUP - runs once at boot (and again every time we wake up from deep sleep,
// since waking from deep sleep is basically the same as a reboot)

void setup() {
  pinMode(PIN_IND_1, OUTPUT);
  pinMode(PIN_IND_2, OUTPUT);
  pinMode(PIN_IND_3, OUTPUT);

  Wire.begin(PIN_ALS_SDA, PIN_ALS_SCL);     // light sensor
  Wire1.begin(PIN_TEMP_SDA, PIN_TEMP_SCL);  // temp sensor
  Wire2.begin(PIN_SPO2_SDA, PIN_SPO2_SCL);  // heart rate sensor

  ledcAttachChannel(PIN_LED_RED,   PWM_FREQ_HZ, PWM_RES_BITS, 0);
  ledcAttachChannel(PIN_LED_GREEN, PWM_FREQ_HZ, PWM_RES_BITS, 1);
  ledcAttachChannel(PIN_LED_BLUE,  PWM_FREQ_HZ, PWM_RES_BITS, 2);

  // touchpad interrupt - fires on PRESS only, we poll for release in serviceTouch()
  touchAttachInterrupt(PIN_TOUCH_WAKE, onTouchPress, TOUCH_THRESHOLD);
}

void loop() {
  if (g_erase_requested) {
    numColorSteps = 0;        // erase_prescription
    g_erase_requested = false;
  }
  g_restart_requested = false; // clear it before we start a fresh run

  runMainFlow();

  // normally runMainFlow() ends by calling goToSleep(), wich resets the chip and we
  // never actually get back here. the only way we DO get here is a touch-restart
  // happened mid flow, in wich case we just loop around and start over from scratch
}