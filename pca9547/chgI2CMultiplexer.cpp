/*******************************************************************************
*  ans = ChgI2CMultiplexer(adrs,ch)                                            *
*  I2Cマルチプレクサー(PCA9547)のチャンネルを切り換える処理                    *
*  この関数は、Arduino標準のWireライブラリを使っています。                     *
*    adrs  : PCA9547のデバイスアドレスを指定する(0x70-0x77)                    *
*    ch    : 切り換えるI2Cバスのチャンネルを指定する(0ch-7ch)                  *
*    ans   : 0=正常終了、それ以外はI2C通信エラーです                           *
*            1=送ろうとしたデータが送信バッファのサイズを超えた(32バイトMAX)   *
*            2=スレーブ・アドレスを送信し、NACKを受信した                      *
*            3=データ・バイトを送信し、NACKを受信した                          *
*            4=その他のエラー                                                  *
*******************************************************************************/
int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
     unsigned char c ;
     int  ans ;

     Wire.beginTransmission(adrs) ;     // 通信の開始
     c = ch & 0x07 ;                    // チャネル(bit0-2)を取り出す
     c = c | 0x08 ;                     // enableビットを設定する
     Wire.write(c) ;                    // Control register の送信
     ans = Wire.endTransmission() ;     // データの送信と通信の終了
     return ans ;
}