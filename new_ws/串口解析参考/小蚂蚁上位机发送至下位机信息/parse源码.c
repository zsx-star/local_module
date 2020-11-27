//接收上层控制器发送的控制请求
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	unsigned char rCh;
	static char rCnt = 0;
	static u8 RecBuf[recBufLen]={0x00};
	uint8_t RecCheckSum=0;
	u8 index;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
	{
		rCh = USART_ReceiveData(USART1);
		RecBuf[rCnt] = rCh;
		if(rCnt == 0)
		{
			rCnt = (0x66 != rCh)?0:rCnt+1;
		}
		else if(rCnt == 1)
		{
			rCnt = (0xcc != rCh)?0:rCnt+1;
		}
		else if(rCnt == 2)
		{
			rCnt = (0x00 != rCh)?0:rCnt+1;
		}
		else if(rCnt == 3)
		{
			rCnt = (0x04 != rCh)?0:rCnt+1;
		}
		else if(rCnt == 4)
		{
			rCnt = (0x5A != rCh)?0:rCnt+1;
		}
		else if(rCnt == 5)
		{
			rCnt++;
		}
		else if(rCnt == 6)
		{
			rCnt++;
		}
		else if(rCnt == 7)
		{
			rCnt = 0;
			for(index=2;index<recBufLen-1;index++)
			{
				RecCheckSum = RecCheckSum+RecBuf[index];		
			}
			if(RecCheckSum == rCh)
			{
				g_brakingVal = RecBuf[5] & 0x7f;
				g_isDriverless = RecBuf[5] & 0x80;
			}
		}
	
	}
} 

