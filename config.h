// cc2500 module connection
// SI = SDIO
// SCK = SCK
// MISO = GD1
// GDO2 = GDO2
// GDO0 = RF1
// CSN = SCS
// PA_EN = TX = TXW
// LNA_EN = RX = RXW
#define CC2500_SPI_GPIO             GPIOB
#define CC2500_SPI_GPIO_RCC         RCC_GPIOB
// LABELED SCK
#define CC2500_SPI_SCK_PIN          GPIO13
// LABELED SPIO
#define CC2500_SPI_MOSI_PIN         GPIO15
// LABELED GIO1
#define CC2500_SPI_MISO_PIN         GPIO14
// LABELED RF2
#define CC2500_SPI_CSN_PIN          GPIO12
#define CC2500_SPI                  SPI2
#define CC2500_SPI                  SPI2
#define CC2500_SPI_DR               SPI2_DR
#define CC2500_SPI_CLK              RCC_SPI2
#define CC2500_SPI_DMA_CLOCK        RCC_DMA1
#define CC2500_SPI_TX_DMA_CHANNEL   DMA_CHANNEL5
#define CC2500_SPI_RX_DMA_CHANNEL   DMA_CHANNEL4

// LABELED RF1
#define CC2500_GDO1_PIN            GPIO8
#define CC2500_GDO1_GPIO           GPIOE

// LABELED RX-W
#define CC2500_LNA_GPIO     GPIOE
#define CC2500_LNA_PIN      GPIO7

// LABELED TX_W
#define CC2500_PA_GPIO     GPIOE
#define CC2500_PA_PIN      GPIO6

// LABELED GIO2
#define CC2500_GDO2_GPIO           GPIOB
#define CC2500_GDO2_PIN            GPIO2

#define NVIC_PRIO_FRSKY      0*64

