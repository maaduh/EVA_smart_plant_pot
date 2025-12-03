# 🌱 Smart Plant Ivy - Sistema de Monitoramento Inteligente de Plantas

Sistema de monitoramento automatizado para plantas usando ESP32 com sensores ambientais, display OLED e alertas visuais RGB.

## 📋 Descrição

O Smart Plant Ivy monitora continuamente as condições da sua planta e fornece feedback visual através de um display OLED com emoticons e LED RGB. O sistema detecta:

- 🌡️ **Temperatura ambiente** (ideal: 15-30°C)
- 💧 **Umidade do solo** via evaporação (>40% = solo úmido)
- ☀️ **Luminosidade** (mínimo: 200 lux)
- 💦 **Nível de água** no reservatório (<150mm = OK)

## 🔧 Componentes Necessários

### Hardware Principal
- **ESP32 DevKit** (qualquer modelo com I2C)
- **Fonte 5V** ou USB para alimentação

### Sensores (Barramento I2C compartilhado)
- **AHT10** - Sensor de temperatura e umidade (0x38)
- **BH1750** - Sensor de luminosidade (0x23)
- **VL53L0X** - Sensor de distância ToF para nível de água (0x29)

### Display e Indicadores
- **SSD1306 OLED 128x64** I2C (0x3C) - Mostra emoticons da planta
- **LED RGB Common Anode** - Alertas visuais por cor

### Outros
- **3x Resistores 220Ω** (para LED RGB)
- Jumpers/fios de conexão
- Protoboard ou PCB

## 🔌 Diagrama de Conexões

### Barramento I2C (Compartilhado por todos os sensores + display)
```
ESP32          Sensores/Display
────────────────────────────────
GPIO 21 (SDA) → SDA (AHT10, BH1750, VL53L0X, OLED)
GPIO 22 (SCL) → SCL (AHT10, BH1750, VL53L0X, OLED)
3.3V          → VCC (todos os sensores e display)
GND           → GND (todos os sensores e display)
```

### LED RGB (Common Anode)
```
ESP32         LED RGB
──────────────────────
3.3V        → Pino I (comum)
GPIO 27     → R (Vermelho) via resistor 220Ω
GPIO 25     → G (Verde) via resistor 220Ω
GPIO 26     → B (Azul) via resistor 220Ω
```

### Endereços I2C
| Dispositivo | Endereço | Função |
|-------------|----------|--------|
| AHT10 | 0x38 | Temperatura e umidade |
| BH1750 | 0x23 | Luminosidade |
| VL53L0X | 0x29 | Distância (nível de água) |
| SSD1306 | 0x3C | Display OLED |

## 🎨 Sistema de Alertas

### Display OLED - Emoticons da Planta
- 😊 **Feliz** (verde) - Todas as condições ideais
- 😰 **Estressada** (amarelo) - 1-2 parâmetros fora do ideal
- 😢 **Triste** (vermelho/roxo) - Situação crítica (≥3 alertas ou água baixa)

### LED RGB
- 🟢 **Ciano/Verde-água** - Tudo OK
- 🟡 **Amarelo** - 1 alerta
- 🔴 **Roxo** - Crítico (múltiplos alertas ou reservatório vazio)

## 🚀 Instalação e Configuração

### Pré-requisitos
- ESP-IDF v5.5+ instalado
- Python 3.11+
- Drivers USB para ESP32

### Compilação e Flash

```bash
# Clone o repositório
git clone https://github.com/artmendess/teste_2.git
cd teste_2

# Configure o projeto ESP-IDF
idf.py set-target esp32

# Compile
idf.py build

# Flash no ESP32 (substitua COM3 pela sua porta)
idf.py -p COM3 flash monitor
```

### Configuração dos Sensores

#### Posicionamento do AHT10 (Sensor de Umidade)
Para detecção de umidade do solo via evaporação:
- Posicione o sensor **5-10cm acima da terra**
- Mantenha próximo à base da planta
- Proteja do sol direto

#### Posicionamento do VL53L0X (Nível de Água)
- Aponte o sensor para a superfície da água no reservatório
- Distância máxima de leitura: ~1200mm
- Alerta quando distância > 150mm (água baixa)

## 📊 Limiares de Monitoramento

| Parâmetro | Ideal | Alerta |
|-----------|-------|--------|
| Temperatura | 15-30°C | Fora dessa faixa |
| Umidade do Solo | >40% | <40% (solo seco) |
| Luminosidade | >200 lux | <200 lux (pouca luz) |
| Nível de Água | <150mm | >150mm (reservatório baixo) |

## 🛠️ Estrutura do Projeto

```
EVE_esp32/
├── main/
│   ├── blink_example_main.c    # Código principal
│   └── CMakeLists.txt
├── components/
│   ├── vl53l0x/                # Driver sensor VL53L0X
│   │   ├── include/
│   │   │   └── vl53l0x.h
│   │   ├── vl53l0x.c
│   │   └── CMakeLists.txt
│   └── ssd1306/                # Driver display OLED
│       ├── include/
│       │   └── ssd1306.h
│       ├── ssd1306.c
│       └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig                    # Configuração ESP-IDF
└── README.md
```

## 🔍 Solução de Problemas

### Sensores não detectados
```bash
# Verifique os endereços I2C conectados
idf.py monitor
# O log mostra um scan I2C na inicialização
```

### VL53L0X timeout constante
- Verifique fiação (SDA/SCL corretos)
- Sensor pode travar após uso prolongado (auto-recovery implementado após 3 timeouts)

### Display OLED não inicializa
- Confirme endereço 0x3C (alguns modelos usam 0x3D)
- Verifique alimentação 3.3V (não 5V!)

## 📝 Características Técnicas

- **Microcontrolador**: ESP32 (160MHz dual-core)
- **Comunicação**: I2C a 100kHz
- **Atualização**: Leituras a cada 1 segundo
- **Display**: 10 segundos entre logs completos
- **Auto-recovery**: VL53L0X reinicia automaticamente após falhas
- **Watchdog**: Reset automático a cada ciclo para evitar travamentos

## 🤝 Contribuindo

Contribuições são bem-vindas! Sinta-se à vontade para:
- Reportar bugs
- Sugerir melhorias
- Adicionar novos sensores
- Melhorar a documentação

## 📄 Licença

Este projeto utiliza componentes com diferentes licenças:
- **Driver VL53L0X**: GPL v3 (baseado em revk/ESP32-VL53L0X)
- **Código principal**: MIT License

## 👨‍💻 Autor

Maria Eduarda Araujo
Desenvolvido com ESP32 - Sistema de monitoramento inteligente de plantas.

## 🙏 Agradecimentos

- Driver VL53L0X baseado no trabalho de [revk/ESP32-VL53L0X](https://github.com/revk/ESP32-VL53L0X)
- Espressif ESP-IDF framework
- Comunidade ESP32 Brasil
- Fundamentos de sistemas embarcados - Renato FCTE (UNB) 2025.2

---

**Status do Projeto**: ✅ Funcional - Monitoramento ativo com recuperação automática de falhas

**Última atualização**: Dezembro 2025
