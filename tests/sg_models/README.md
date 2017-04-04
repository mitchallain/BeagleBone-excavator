# SG Models Directory

This directory is for IFAC style deterministic subgoal models. It was created on April 03, 2017, while writing the journal/tech brief extension of the IFAC paper.

## Importing sg_models from the tests directory
Importing for a tests/ script such as blended_old.py requires the following statement:

`from sg_models.sgs_mmdd import sg_model`

where 'mmdd' is the month and day codes corresponding to the date of creation.

The directory is considered a package, and we are importing a variable from a module within that package. Using variable imports is not perfect, but it makes it easy to edit the stored data and no parsing is necessary.