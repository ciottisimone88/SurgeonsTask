msg.out <- capture.output(suppressMessages(library(tidyverse)))

stimuli <- expand.grid (
  surface = c(1,2),
  x = c(-1,0,1),
  y = c(-1,0,1),
  multimodal = c(0, 1),
  repetitions = seq(1,10)
)

N <- nrow(stimuli)
trial_order <- seq(1:N) %>%
sample(size = N)

stimuli_random <- stimuli[trial_order,] %>%
  dplyr::select(-repetitions)

write_csv(stimuli_random, "stimuli.csv")
