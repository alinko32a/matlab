
mypi = raspi;

%% mpu9250
%計算で使用するので，選択したレンジを入力する
accRange = 16.0;
%%計算で使用するので，選択したレンジを入力する
gyroRange = 2000.0;

mpu9250_dev = i2cdev(mypi,'i2c-1','0x68');
%スリープモードを解除
write(mpu9250_dev,[hex2dec( '6b' ) hex2dec( '00' )]);
%加速度センサの測定レンジの設定
write(mpu9250_dev,[hex2dec( '1c' ) hex2dec( '18' )]);
%ジャイロセンサの測定レンジの設定
write(mpu9250_dev,[hex2dec( '1b' ) hex2dec( '18' )]);
%bypass mode(磁気センサが使用出来るようになる)
write(mpu9250_dev,[hex2dec( '37' ) hex2dec( '02' )]);
%磁気センサのAD変換開始
ak8963_dev = i2cdev(mypi,'i2c-1','0x0c');
write(ak8963_dev,[hex2dec( '0a' ) hex2dec( '16' )]);


write(mpu9250_dev,hex2dec( '3b' ));
accGyroTempData = read(mpu9250_dev,14);

write(ak8963_dev,hex2dec( '02' ));
ST1Bit = read(ak8963_dev,1);
%if (ST1Bit == 1)
    write(ak8963_dev,hex2dec( '03' ));
    magneticData = read(ak8963_dev,7);
%end

%% AGB65-RSC
myserialdevice = serialdev(mypi,'/dev/ttyS0');
%180°モード
write(myserialdevice,255);
write(myserialdevice,3);
write(myserialdevice,1);
write(myserialdevice,5);

clear;