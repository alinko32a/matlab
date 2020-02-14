/*******************************************************************************
*  ans = ChgI2CMultiplexer(adrs,ch)                                            *
*  I2C�}���`�v���N�T�[(PCA9547)�̃`�����l����؂芷���鏈��                    *
*  ���̊֐��́AArduino�W����Wire���C�u�������g���Ă��܂��B                     *
*    adrs  : PCA9547�̃f�o�C�X�A�h���X���w�肷��(0x70-0x77)                    *
*    ch    : �؂芷����I2C�o�X�̃`�����l�����w�肷��(0ch-7ch)                  *
*    ans   : 0=����I���A����ȊO��I2C�ʐM�G���[�ł�                           *
*            1=���낤�Ƃ����f�[�^�����M�o�b�t�@�̃T�C�Y�𒴂���(32�o�C�gMAX)   *
*            2=�X���[�u�E�A�h���X�𑗐M���ANACK����M����                      *
*            3=�f�[�^�E�o�C�g�𑗐M���ANACK����M����                          *
*            4=���̑��̃G���[                                                  *
*******************************************************************************/
int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
     unsigned char c ;
     int  ans ;

     Wire.beginTransmission(adrs) ;     // �ʐM�̊J�n
     c = ch & 0x07 ;                    // �`���l��(bit0-2)�����o��
     c = c | 0x08 ;                     // enable�r�b�g��ݒ肷��
     Wire.write(c) ;                    // Control register �̑��M
     ans = Wire.endTransmission() ;     // �f�[�^�̑��M�ƒʐM�̏I��
     return ans ;
}