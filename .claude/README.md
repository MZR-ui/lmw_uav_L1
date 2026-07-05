# .claude Directory Structure

This directory contains Claude Code workspace files for managing plans, skills, agents, and artifacts.

## Directory Layout

- **plans/**: Implementation plans created during plan mode
- **skills/**: Custom skills and commands for this workspace
- **agents/**: Agent configurations and sub-agent definitions
- **artifacts/**: Generated files, diagrams, and documentation
- **logs/**: Execution logs and debugging information

## Usage

These directories are automatically used by Claude Code during operation:
- Plans are stored when using plan mode for complex tasks
- Skills can be invoked via `/skill-name` commands
- Agents are spawned for parallel exploration and implementation
- Artifacts store intermediate outputs and generated content
- Logs capture execution history for debugging

## Notes

- This directory is managed by Claude Code
- Files here persist across sessions
- Do not manually edit unless you understand the structure
