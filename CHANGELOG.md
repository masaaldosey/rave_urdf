# Changelog

All notable changes to this project will be documented in this file. The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## Upcoming changes
- unit tests.
- script to enable generate an `IKFast` model for the given robot.
- enable loading of *URDF*s containing more than one robot.

## `v0.0.1` [2024-10-16]

### Added

- a plugin which loads a given *URDF* file into openRAVE as `OpenRAVE::RobotBase`.
- however, the user needs to ensure that the *URDF* has only one robot.
- exploits builtin openRAVE helpers for *Collada* export.
