/*
 *
 *
 *
 *
 *
 *
 */
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
 #include <string.h>

#include "rfid.h"

int fd;

/**
 *@brief RC522读取寄存器数据
 *@return 读取寄存器的数据
 */
static char RC522Read(int addr){
	unsigned char ReData = 0;
	unsigned char ret = 0;
	addr = addr << 1;
	addr = addr | (1 << 7);
	addr = addr & ~(1 << 0);
	
	ret = write(fd , &addr , 1);
	if(ret < 0)
		printf("write register failed \n");
	ret = read(fd , &ReData , 1);
	if(ret < 0)
		printf("write register failed \n");
	return (char )ReData ;
}
/**
 *@brief RC522写数据到指定寄存器
 *@return 写入数据的长度
 */
static int RC522Write(int addr,int data){
	
	unsigned char TxBuf[2] = {0};
	addr = addr << 1;
	addr = addr & ~(1 << 7);
	addr = addr & ~(1 << 0);
	TxBuf[0] = addr ;
	TxBuf[1] = data ; 
	write(fd , TxBuf , sizeof(TxBuf));
	return 0;
}
/**
 *@brief 设置RC522寄存器位
 *@param reg:寄存器地址		mask:置位值
 */
static void RC522SetBitMask(unsigned char reg,unsigned char mask){
	char tmp = 0 ;
	tmp = RC522Read(reg) | mask ;
	RC522Write(reg  , tmp);
}
/**
 *@brief 清除RC522寄存器位
 *@param reg:寄存器地址		mask:清位值
 */
static void RC522ClearBitMask(unsigned char reg,unsigned char mask){
	char tmp = 0;
	tmp = RC522Read(reg) & ~(mask);
	RC522Write(reg , tmp);
}
/**
 *@brief RC522开启天线
 */
static void Rc522AntennaOn(void){
	
	unsigned char i = 0;
	RC522Write(TxASKReg,0x40);
	
	i = RC522Read(TxControlReg);
	if(!(i&0x03))
	{
		RC522SetBitMask(TxControlReg,0x03);
	}
}
/**
 *@brief RC522模块初始化
 *@return 成功：0	失败：-1
 */
static int Rc522Init(){
	char version = 0;
	//reset 
	RC522Write(CommandReg,PCD_RESETPHASE);
	RC522Write(ModeReg,0x3D);
	RC522Write(TReloadRegL, 30);
	RC522Write(TReloadRegH, 0);
	RC522Write(TModeReg, 0x8D);
	RC522Write(TPrescalerReg, 0x3E);
	
	version = RC522Read(VersionReg);
	printf("Chip Version:0x%x\n",version);
	
	Rc522AntennaOn();
	return;
}
/**
 *@brief 通过RC522和ISO14443卡通讯
 *@param Command:RC522命令字
 *       pInData[IN]:通过RC522发送到卡片的数据
 *       InLenByte[IN]:发送数据的字节长度
 *       pOutData[OUT]:接收到的卡片返回数据
 *       *pOutLenBit[OUT]:返回数据的位长度
 *@return 		 
 */
static char RC522ComMF522(unsigned char Command,unsigned char *pInData,unsigned char InLenByte,unsigned char *pOutData,unsigned int *pOutLenBit){
	unsigned char irqEn  = 0x00;
	unsigned char waitFor = 0x00;
	char status = MI_ERR;
	unsigned char lastBits = 0;
	unsigned char n = 0;
	int i = 0;
	switch (Command)
	{
		case PCD_AUTHENT:
	  		irqEn   = 0x12;
			  waitFor = 0x10;
			  break;
		case PCD_TRANSCEIVE:
	  		irqEn   = 0x77;
	  		waitFor = 0x30;
	  		break;
		default:
	  		break;
	}
	
	RC522Write(ComIEnReg, irqEn|0x80);
	RC522ClearBitMask(ComIrqReg, 0x80);
	RC522Write(CommandReg, PCD_IDLE);
	RC522SetBitMask(FIFOLevelReg, 0x80); // 清空FIFO
	for(i = 0; i < InLenByte; i ++)
		RC522Write(FIFODataReg, pInData[i]); // 数据写入FIFO 
	
	RC522Write(CommandReg, Command); // 命令写入命令寄存器
	
	if(Command == PCD_TRANSCEIVE)
		RC522SetBitMask(BitFramingReg,0x80); // 开始发送 
	
	i = 6000; //根据时钟频率调整，操作M1卡最大等待
  	do {
		n = RC522Read(ComIrqReg);
		i--;
	}while((i != 0)&&!(n & 0x01)&&!(n & waitFor));
	
	RC522ClearBitMask(BitFramingReg, 0x80);
	
	if ( i != 0){
		if(!(RC522Read(ErrorReg) & 0x1B)){
			status = MI_OK;
			if (n&irqEn&0x01)
				status = MI_NOTAGERR;
			if(Command == PCD_TRANSCEIVE){
				n = RC522Read(FIFOLevelReg);//读取到的数据字节数
				lastBits = RC522Read(ControlReg) & 0x07;
				if(lastBits)
					*pOutLenBit = (n-1)*8 + lastBits;
				else
					*pOutLenBit = n*8;
				
				if(n == 0)
					n = 1;
				if(n>MAXRLEN)
					n = MAXRLEN;
				
				for (i=0; i<n; i++)
					pOutData[i] = RC522Read(FIFODataReg); 
			}
		}else{
			status = MI_ERR;
		}
	}
	
	RC522SetBitMask(ControlReg, 0x80);// stop timer now
	RC522Write(CommandReg, PCD_IDLE); 
	
	return status;
}
/**
 *@brief 寻卡
 *@param reg_code:寻卡方式 0x52 = 寻感应区内所有符合1443A标准的卡
 *					  0x26 = 寻未进入休眠的卡
 *		 pTagType:卡片类型代码
 *@return 成功返回MI_OK
 */
char Rc522Request(unsigned char req_code,unsigned char *pTagType){
	char status;
	unsigned int unLen;
	unsigned char ucComMF522Buf[MAXRLEN]; 
	RC522ClearBitMask(Status2Reg, 0x08);//温度传感器位
	RC522Write(BitFramingReg, 0x07);//清除读取到的最后一个字节位数
	RC522SetBitMask(TxControlReg, 0x03);
	
	ucComMF522Buf[0] = req_code;
	
	status = RC522ComMF522( PCD_TRANSCEIVE , ucComMF522Buf , 1 , ucComMF522Buf , &unLen);
	if((status == MI_OK) && (unLen == 0x10)){
    	*pTagType     = ucComMF522Buf[0];
    	*(pTagType+1) = ucComMF522Buf[1];
  	}else{
		status = MI_ERR;
  	}
	return status;
}

/**
 *@bfief 防冲撞
 *@param pSnr:卡片序列号
 *@return 成功：MI_OK
 */
char Rc522Anticoll(unsigned char *pSnr){
	char status = 0;
	int i = 0;
	unsigned int unLen = 0;
	unsigned char snr_check = 0; 
	unsigned char ucComMF522Buf[MAXRLEN] = {0}; 
	RC522ClearBitMask(Status2Reg, 0x08);
	RC522Write(BitFramingReg, 0x00);
	RC522ClearBitMask(CollReg, 0x80);
	
	ucComMF522Buf[0] = PICC_ANTICOLL1;
	ucComMF522Buf[1] = 0x20;
	
	status = RC522ComMF522(PCD_TRANSCEIVE, ucComMF522Buf,2, ucComMF522Buf, &unLen);
	if(status == MI_OK){
		for(i = 0; i < 4 ;i ++){
			*pSnr++ = ucComMF522Buf[i];
			snr_check ^= ucComMF522Buf[i];
		}
		if(snr_check != ucComMF522Buf[4]){
			status = MI_ERR;
		}
	}
	RC522SetBitMask(CollReg,0x80);
	
	return status;
}
/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
unsigned long const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
static unsigned long crc16_byte(unsigned long crc, const unsigned char data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}
/**
 * crc16 - compute the CRC-16 for the data buffer
 * @crc:	previous CRC value
 * @buffer:	data pointer
 * @len:	number of bytes in the buffer
 *
 * Returns the updated CRC value.
 */
unsigned long crc16(unsigned long crc, unsigned char const *buffer, int len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}

/**
 *@brief 用于MF522计算CRC16
 *@param pIn:计算数据  len：数据长度  pOut:计算之后的数据
 */
static void CalulateCRC(unsigned char *pIn,unsigned char len,unsigned char *pOut)
{
	unsigned char i,n;

	RC522ClearBitMask(DivIrqReg,0x04);
	RC522Write(CommandReg,PCD_IDLE);
	RC522SetBitMask(FIFOLevelReg,0x80);

	for(i = 0;i < len;i++)
	{
		RC522Write(FIFODataReg,*(pIn + i));
	}
	RC522Write(CommandReg,PCD_CALCCRC);
	i = 0xFF;
	do
	{
		n = RC522Read(DivIrqReg);
		i--;
	}while((i != 0) && !(n&0x04));
	
	pOut[0] = RC522Read(CRCResultRegL);
	pOut[1] = RC522Read(CRCResultRegM);
}
/**
 *@brief 选定卡片
 *@param pSnr:卡片序列号，4字节
 *@reutrn 成功：MI_OK
 */
char Rc522Select(unsigned char *pSnr){
	char status = 0 ;
	unsigned char i = 0;
	unsigned int unLen = 0;
	unsigned char ucComMF522Buf[MAXRLEN] = {0};
	ucComMF522Buf[0] = PICC_ANTICOLL1;
	ucComMF522Buf[1] = 0x70;

	for(i = 0; i < 4; i ++){
		ucComMF522Buf[i + 2] = pSnr[i];
		ucComMF522Buf[6] ^= pSnr[i];
	}
	CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
	//crc16(0 ,ucComMF522Buf, 7);
	RC522ClearBitMask(Status2Reg,0x08);
	status = RC522ComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
	if((status == MI_OK) && (unLen == 0x18))
	{
		status = MI_OK;
	}
	else
	{
		status = MI_ERR;
	}
	return status;
}
/**
 *@brief 验证卡片密码
 *@param auto_mode:密码验证模式  0x60 = 验证A密钥
 *								 0x61 = 验证B密钥
 *		 addr:地址块
 *		 pKey:密码
 *		 pSnr:卡片序列号，4字节
 *@return 成功：MI_OK
 */
char Rc522AuthState(unsigned char auto_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr){
		char status;
	unsigned int unLen;
	unsigned char i,ucComMF522Buf[MAXRLEN];
	
	ucComMF522Buf[0] = auto_mode;
	ucComMF522Buf[1] = addr;
	
	memcpy(&ucComMF522Buf[2], pKey , 6);
	memcpy(&ucComMF522Buf[8] , pSnr , 4);
	
	status = RC522ComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
	if((status != MI_OK) || (!(RC522Read(Status2Reg)&0x08)))
	{
		status = MI_ERR;
	}
	return status;
}
/**
 *@brief 读取卡一块的数据
 *@param addr:块地址
 *       p:读出的数据,16字节
 *@return 成功：MI_OK
 */
char Rc522ReadBlock(unsigned char addr ,unsigned char *p)
{
	char status;
	unsigned int unLen;
	unsigned char i,ucComMF522Buf[MAXRLEN];

	ucComMF522Buf[0] = PICC_READ;
	ucComMF522Buf[1] = addr;

	CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

	status = RC522ComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
	if((status == MI_OK) && (unLen == 0x90))
	{
		for(i = 0;i < 16;i++)
		{
			*(p + i) = ucComMF522Buf[i];
		}
	}
	else
	{
		status = MI_ERR;
	}
	return status;
}
/**
 *@brief 写数据到卡的一块
 *@param addr:块地址
 *       p:写入的数据,16字节
 *@return 成功：MI_OK
 */
char Rc522WriteBlock(unsigned char addr ,unsigned char *p)
{
	char status;
	unsigned int unLen;
	unsigned char i,ucComMF522Buf[MAXRLEN];

	ucComMF522Buf[0] = PICC_WRITE;
	ucComMF522Buf[1] = addr;
	CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

	status = RC522ComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
	if((status != MI_OK) || (unLen != 4) || (ucComMF522Buf[0] & 0x0F) != 0x0A)
	{
		status = MI_ERR;
	}

	if(status == MI_OK)
	{
		for(i = 0;i < 16;i++)
		{
			ucComMF522Buf[i] = *(p + i);
		}
		CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

		status = RC522ComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
		if((status != MI_OK) || (unLen != 4) || (ucComMF522Buf[0] & 0x0F) != 0x0A)
		{
			status = MI_ERR;
		}
	}
	return status;
}
int main(){
	unsigned char UID[4] = { 0 };
	unsigned char temp[2] = { 0 };
	char ret = 0;
	char a = 0;
	int i;
	unsigned char Key[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	unsigned char ReadBuf[16] = { 0 };
	unsigned char WriteBuf[16] = "wangxiaohui";
	fd = open("/dev/spidev2.0",O_RDWR);
	printf("fd = %d \n",fd);
	
	Rc522Init();
	
	RC522Write(0x09,0xaa);
	a = RC522Read(0x09);
	printf("read a = 0x%x \n" , a);
	
	ret = Rc522Request(0x52, temp);
	if(ret == MI_ERR){
		printf("Rc522Request faild \n");
	}
	printf("Rc522Request : temp[0] = %x.temp[1] = %x\n",temp[0],temp[1]);
	Rc522Anticoll(UID);
	printf("Rc522Anticoll : UID = %x %x %x %x .\n" , UID[0] , UID[1] , UID[2] , UID[3] );
	ret = Rc522Select(UID);
	if(ret == MI_ERR){
		printf("Rc522Select faild \n");
	}
	ret = Rc522AuthState(PICC_AUTHENT1A , 13 , Key , UID);
	if(ret == MI_ERR){
		printf("Rc522AuthState faild \n");
	}
	Rc522WriteBlock(13,WriteBuf);
	
	Rc522ReadBlock( 13 , ReadBuf);
	for( i = 0; i < 16 ; i ++){
		printf("%c",ReadBuf[i]);
	}
	printf("\n");
	
	
	
	return 0;
}










