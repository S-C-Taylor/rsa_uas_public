# rsa_uas
Repository for our COMP3431 major project

## ARDrone Autonomy Install
sudo apt-get install ros-indigo-ardrone-autonomy

## Linux Build

```
sudo apt-get install libncurses5-dev
cd (sdk folder)/Examples/Linux
sed -i '131s/$/ -lm/' Navigation/Build/Makefile
sed -i '47s/$/ -lgdk-x11-2.0 -lgobject-2.0 -lm/' video_demo/Build/Makefile
make
```

## SDK

http://developer.parrot.com/products.html ([Direct Download](http://developer.parrot.com/docs/SDK2/ARDrone_SDK_2_0_1.zip))

## Useful Links

Sensefly Autonomous Landing: https://stisrv13.epfl.ch/masters/img/645.pdf

Feature Detection: https://www.google.com.au/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwiD1s6_tI_WAhWHTrwKHTm5ChgQFgg0MAE&url=http%3A%2F%2Fwww.mdpi.com%2F1424-8220%2F15%2F7%2F14887%2Fpdf&usg=AFQjCNEelo3VSpasb2rVhUnryZPkLqVkeg
