import sys
from dulwich import porcelain

# === COMMANDS ===
# To clone:  python git_sync.py clone git@github.com:TeamName/Repo.git
# To pull:   python git_sync.py pull
# To commit: python git_sync.py push "Commit message"

action = sys.argv[1].lower() if len(sys.argv) > 1 else "help"

if action == "clone":
    repo_url = sys.argv[2]
    print(f"Cloning {repo_url}...")
    porcelain.clone(repo_url)
    print("Done!")

elif action == "pull":
    print("Pulling latest code...")
    porcelain.pull(".", "origin")
    print("Up to date!")

elif action == "push":
    msg = sys.argv[2] if len(sys.argv) > 2 else "Update from school laptop"
    print("Adding and committing changes...")
    porcelain.add(".")
    porcelain.commit(".", message=msg.encode('utf-8'))
    print("Pushing to GitHub...")
    porcelain.push(".", "origin", "refs/heads/main")  # Change 'main' to 'master' if your repo uses master
    print("Pushed successfully!")

else:
    print("Usage:\n  python git_sync.py clone <SSH_URL>\n  python git_sync.py pull\n  python git_sync.py push 'message'")