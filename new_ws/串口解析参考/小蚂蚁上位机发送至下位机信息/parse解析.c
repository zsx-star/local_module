//接收上层控制器发送的控制请求
void USART1_IRQHandler(void)                	          //串口1中断服务程序  串口中断一次就进一次串口中断函数
{
	unsigned char rCh;                                    // char型一个字节（代表每次从串口读取的那个字节）
	static char rCnt = 0;							      // 定位索引
	static u8 RecBuf[recBufLen]={0x00};
	uint8_t RecCheckSum=0;
	u8 index;
	unsigned char rCh1; 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)   // 进中断的标志
	{
		rCh = USART_ReceiveData(USART1);                  // 每次接收一个字节的数据（8个比特）
		RecBuf[rCnt] = rCh;
		if(rCnt == 0)								      // 第1个字节判断是否0x66  RecBuf[0] 
		{
			rCnt = (0x66 != rCh)?0:rCnt+1;				  // 是的话继续往下判断
		}
		else if(rCnt == 1)
		{
			rCnt = (0xcc != rCh)?0:rCnt+1;             	 // 第2个字节判断是否0xcc   RecBuf[1] 
		}
		else if(rCnt == 2)
		{
			rCnt = (0x00 != rCh)?0:rCnt+1;               // 包长度分配了2个字节的大小，但第1个字节用不到  RecBuf[2] 
		} 
		else if(rCnt == 3)
		{
			rCh1 == RecBuf[3];
			rCnt = (rCh1 != rCh)?0:rCnt+1;               // 第2个字节表示真实的包长度  RecBuf[3] 
		}
		else if(rCnt == 4)
		{
			rCnt = (0x5A != rCh)?0:rCnt+1;               // 找到数据头0x5A  RecBuf[4] 
		}
		else if(rCnt == 5)
		{	
			for(int i = 0;i < rCh1 - 2)                   // 用rCh1为媒介来暂时保存数据包的长度，
				rCnt++;									  // 减去包ID和校验位两个字节，
		}			
		else if(rCnt == rCh1 + 4 - 1)                     // 加上开头的66cc和数据包占的字节数再减去1，即找到校验位
		{
			rCnt = 0;
			for(index=2;index<recBufLen-1;index++)        // 进行校验（）
			{
				RecCheckSum = RecCheckSum+RecBuf[index];  // 从RecBuf[2]开始加到RecBuf[6]作为校验和	
			}
			if(RecCheckSum == rCh)                        // 如果校验成功，开始读取数据
			{
				g_brakingVal = RecBuf[5] & 0x7f;          // RecBuf[5]的低7位  存放制动信号
				g_isDriverless = RecBuf[5] & 0x80;        // RecBuf[5]的高1位  存放驾驶模式信号
			}
		}
	
	}
} 
		// else if(rCnt == 5)                               // 此下直接往后跳过2个字节，找到校验位
		//{
			//rCnt++;                                      
		//}
		//else if(rCnt == 6)
		//{
			//rCnt++;
		//}
