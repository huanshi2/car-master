# car
2019.4  
4.17  
激光传感器还存在问题，默认地址0x52可以使用，修改i2c地址只能修改一个，三个的情况下，初始化完第三个之后，第二个会失效  
4.18  
把原店家源码上的mpu部分移植了过来，可以读取欧拉角，但是似乎有点不准，而且也还没知道怎么设置0角度。  
4.27
把mpu和避障的都调通了

