Temos três missões sendo executadas:
- Simple_mission_avgas: modelagem do avião as is.
- Simple_mission_ethanol: altera apenas o consumo específico de combustível (devido à mudança de avgas -> etanol) para analisar o impacto no alcance e no desempenho de modo geral da aeronave.
- Simple_mission_optimized configuration: alteração dos parâmetros (afilamento, alongamento e espessura relativa da asa) para refletir a configuração otimizada.

As três missões foram utilizadas para avaliar o gráfico de fuel_burnt do SUAVE e comparar os alcances das configurações de acordo com ele. 

Os parâmetros para a Simple_mission_optimized foram encontrados a partir da execução do arquivo Optimization_ethanol. Esse arquivo utiliza o otimizador SLSQP, com as restrições e a função objetivo descritas no TCC. A otimização considera apenas o segmento de cruzeiro, utilizando a equação de Breguet. 

Alguns pontos: 
- O arquivo Optimization_ethanol gera gráficos de cada uma das variáveis da otimização pelo alcance. Os pontos marcados como "sensibilidade" mostram como o alcance varia em relação ao parâmetro em questão. Já os pontos marcados como "otimização" mostram os pontos de fato avaliados pelo otimizador. 
  <img width="2552" height="1380" alt="image" src="https://github.com/user-attachments/assets/a744e610-4f02-4bc7-9f69-c7cc3f127c6d" />
  <img width="2521" height="1322" alt="image" src="https://github.com/user-attachments/assets/97bfd488-7c98-4cfb-b8f7-6f6158c53ffb" />
  <img width="2514" height="1333" alt="image" src="https://github.com/user-attachments/assets/80d9cd8f-5b9a-4f27-ae35-dcead5965baf" />
  
- A função fuel_burnt considera o combustível queimado em todos os segmentos da missão, não apenas na fase de cruzeiro. 



