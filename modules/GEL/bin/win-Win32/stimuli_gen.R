msg.out <- capture.output(suppressMessages(library(tidyverse)))

stencil_points <- 5

stimuli <- expand.grid (
  surface = c(1,2),
  x = c(-1,0,1),
  y = c(-1,0,1),
  multimodal = c(0, 1),
  repetitions = seq(1,10),
  is_training = c(0)
)

N <- nrow(stimuli)
trial_order <- seq(1:N) %>%
sample(size = N)

stimuli_random <- stimuli[trial_order,] %>%
  dplyr::select(-repetitions) %>%
  add_row(surface     = c(1,1,2,2,1,1,2,2),
          x           = c(0,0,0,0,-1,1,-1, 1),
          y           = c(0,0,0,0,-1,1, 1,-1),
          multimodal  = c(0,1,0,1,0,1,0,1),
          is_training = c(1,1,1,1,1,1,1,1),
          .before = 1)

if (stencil_points == 5) {
  stimuli_random <- stimuli_random %>%
    dplyr::filter((x == 0 & y == 0) | (x != 0 & y != 0))
}

write_csv(stimuli_random, "stimuli.csv")
