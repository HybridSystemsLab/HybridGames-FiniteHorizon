# HybridGames-FiniteHorizon
Simulation M-files for examples in CDC'22 paper:  Sufficient Conditions for 
Optimality in Finite-Horizon Two-Player Zero-Sum Hybrid Games

Author: Santiago Jimenez Leudo
Revision: 0.0.0.1 Date: 04/05/2022 16:32:00
https://github.com/HybridSystemsLab/HybridGames-FiniteHorizon

Requirements: Matlab (Developed in R2020b)
Install HyEQ Toolbox (Beta) v3.0.0.22 either by double clicking 
on the 'Hybrid Equations Toolbox' file or by downloading it from 
https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta
Look up for the right version in 'View Version History'

Content: (Download as a folder and add it to the Path in Matlab)
  - Hybrid Equations Toolbox
  - MotivExZeroSumFHHyGame
  - Ex1D2PZeroSumFHHyGame.m
  - ExJumpsActBouncingBallUnderAttack.m
  - BouncingBallZSGame.m
  - SaddlePointBouncingBall.m

----------------------------------------------------------------------------
Motivational Example
Run 'MotivExZeroSumFHHyGame.m'

It will create a simulation for the hybrid system in (3) for the following 
parameters:
  a=-1,
  b_1=b_2=1,
  \delta=\xi=2,
  \mu=1,
  \sigma=0.5,
  Q_C=1,
  R_{C1}=1.304,
  R_{C2}=-4,
  P=0.4481
  
They are defined and can be modified in the 'Initialization' section.  
  
As a result, 
  - it will create the hybrid solution '(solphih,soluh,sollwh)', with hybrid 
    time '(t,jv)', and running cost 'J' when jumps are triggered at x=\mu;
  - it will generate a plot (Figure 1) including:
      1. Response of hybrid solution (blue and red) and response of continuous 
          solution (green).
      2. Input actions of hybrid solution (blue and red) and input actions of 
          continuous solution (green).
      3. Running cost of hybrid solution (blue and red), running cost of 
          continuous solution (green), and optimal cost (black) calculated by 
          applying Theorem 3.6.
    
Example 3.7 (Hybrid game with nonunique solutions)    
Run 'Ex1D2PZeroSumFHHyGame.m'                      
    
In addition to the motivational example plot, 
    
  - it will create the continuous solution '(phik, uk,wk)', with time 't', 
    and running cost 'Jk' when it evolves from x=\mu via flow;
  - it will generate a plot (Figure 2) including:
      1. Response of hybrid solution (blue and red) and response of continuous 
          solution (green).
      2. Input actions of hybrid solution (blue and red) and input actions of 
          continuous solution (green).
      3. Running cost of hybrid solution (blue and red), running cost of 
          continuous solution (green), and optimal cost (black) calculated by 
          applying Theorem 3.6.
          
----------------------------------------------------------------------------
Example 3.8 (Boucing ball)
Run 'ExJumpsActBouncingBallUnderAttack.m'
Associated files: 'BouncingBallZSGame.m' and 'SaddlePointBouncingBall.m'

It will create a simulation for the hybrid system in (44) for the following 
parameters:
  \lambda=0.8,
  R_{D1}=10,
  R_{D2}=-20,
  Q_D=0.189,
  x(0)=[1;1]
  
They are defined and can be modified in the 'Initialization' section.  
  
As a result, 
  - it will create the hybrid system in (18), calling it 'system_bb' as an
    instance of 'BouncingBallZSGame.m', which invokes the class 'HybridSystem'
    of the HyEQ Toolbox (see line 86); 
  - it will generate a plot (Figure 3) including:
      1. Position of optimal solution,
      2. Velocity of optimal solution,
      3. Optimal input control action,
      4. Optimal input attack action,
      3. Running cost of optimal solution (blue and red) and optimal cost (black)
        calculated by applying Theorem 3.6.
  - it will create a set of nonoptimal solutions '(phit,un,wn)', with hybrid 
    time '(t,jt)', and running cost 'Jt', by running 'SaddlePointBouncingBall.m'.
    Therein, the optimal feedbak laws are multiplied by varying factors,
    epsilonu and epsilonw, calculating the cost associated by this scalation,
    and generating a plot in 3D (Figure 4) of the behavior of the cost vs the scalation factors.
    See in red the optimal cost and the saddle behavior around it.
