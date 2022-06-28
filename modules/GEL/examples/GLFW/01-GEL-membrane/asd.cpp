if (next_trial) {
            trial_idx++;
            if (trial_idx >= stimuli.size()) glfwSetWindowShouldClose(window, GLFW_TRUE);
            else {
                active_surface      = stimuli[trial_idx][0]-1;
                active_point_x      = stimuli[trial_idx][1];
                active_point_y      = stimuli[trial_idx][2];
                multimodal_feedback = stimuli[trial_idx][3];
            }

            active_point_sphere->setLocalPos(8.0 * kGEMSphereRadius * active_point_x, 8.0 * kGEMSphereRadius * active_point_y, kGEMSphereRadius);
            
            std::cout << "IDX: " << trial_idx << "\n";
            std::cout << "Active Surface: " << active_surface << "\n";
            std::cout << "Active Point X: " << active_point_x << "\n";
            std::cout << "Active Point Y: " << active_point_y << "\n";
            std::cout << "Multi-modal Feedback: " << multimodal_feedback << "\n\n";

            next_trial = false;
        }
