# Truss Design Calculator

The following repository enables engineers to easily calculate the internal force, stress and strain of each member within any 2-D truss design. The code takes in the following inputs:

1. **Nodes**: A single point that represents a joint. 
2. **Members**: The beams that connect your nodes. 
3. **Supports**: The truss supports. Configure the support to have x and y components. 
4. **Materials**: The materials that make up your beams. Materials are defined by their cross sectional area and young's modulus. 
5. **Loads**: The external loads that act on your truss. Loads are defined by their magnitude and direction. 

## Example
The code is configured for the following example. Each member is made out of steel. 
![Diagram](truss-example.png)