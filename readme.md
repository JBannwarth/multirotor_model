# New Multirotor Model - Project Requirements
By: J. X. J. Bannwarth, 30/06/16

## List of Requirements

Model must:

- Be well encapsulated - each component should be easily replaceable
- Be arbitrarily scalable - it should be easy to change from quadrotor to hexarotor to octorotor configuration (or any other feasible configuration)
- Use quaternions to cover any possible use case
- Include conversion blocks to run with quaternion or Euler controllers
- Run efficiently (ideally perform a ~60s simulation in 2-3 real time s)
- Be able to run in batch simulations
- Include comprehensive data logging and plotting
- Include ways to plot against experimental data

Code must:
- Be written using consistent naming conventions
	- Use underscore style for variables `variable_name`
	- Spell out greek letters `xi`, `omega`, etc.
	- Use `_vec` and `_mat` to denote vectors and matrices
	- Add contact address, name of creator, and date of modification to all files
	- Add paper reference/doi/url to file description for algorithms/models
- Simulink model must be clean and easy to understand
	- Name all lines with appropriate variable names
	- Name all block input/output appropriately
	- Use colour coding to match blocks of similar function
	- Add comment blocks to explain what is happening
- Be well organised in folders
- Be backed up on GitHub and kept up to date
