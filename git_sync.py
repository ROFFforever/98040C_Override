import sys
from dulwich import porcelain

# === COMMANDS ===
# To clone:  python git_sync.py clone git@github.com:TeamName/Repo.git
# To pull:   python git_sync.py pull
# To commit: python git_sync.py push "Commit message"

REPO = "."

HELP = """Usage: python git_sync.py <command> [args]

Getting started:
  clone <url> [dir]            copy a remote repo
  init                         make a new repo in this folder

Saving work:
  status                       show changed / staged / untracked files
  add <paths...>               stage files ("." stages everything)
  commit -m "msg" [-a] [--amend]   commit staged changes (-a stages tracked changes first)
  push "msg"                   stage everything, commit with msg, push (original behavior)
  publish [remote] [branch]    plain push of the current branch, no commit
  rm <paths...> [--cached]     delete files / stop tracking them
  mv <src> <dst>               move or rename a file

Looking around:
  log [-n N] [--oneline] [--stat] [-p] [--author X] [--grep X] [paths...]
  show [commit/tag...]         show a commit
  diff [--staged] [commit] [commit2] [-- paths...]
  ls-files                     list tracked files
  blame <file>                 who changed each line
  rev-parse <name>             turn a branch/tag/HEAD into a full hash

Branches and tags:
  branch                       list branches
  branch <name> [start]        create a branch
  branch -d <name>             delete a branch
  checkout <branch|commit>     switch to it
  checkout -b <name>           create and switch to a new branch
  switch <branch> / switch -c <name>
  merge <branch> [--no-ff]
  tag                          list tags
  tag <name> [-m "msg"]        create a tag (-m makes it annotated)
  tag -d <name>                delete a tag

Undoing:
  reset [--soft|--mixed|--hard] [commit]
  restore <paths...> [--staged] [--source=<commit>]
  revert <commit>
  cherry-pick <commit>
  clean                        delete untracked files
  stash [push|pop|list]

Remotes:
  remote [-v]                  list remotes
  remote add <name> <url>
  remote remove <name>
  fetch [remote]
  pull [remote]
"""


def flag(args, *names):
    found = False
    for n in names:
        while n in args:
            args.remove(n)
            found = True
    return found


def option(args, *names):
    for n in names:
        if n in args:
            i = args.index(n)
            if i + 1 >= len(args):
                print(f"Missing value for {n}")
                sys.exit(1)
            value = args[i + 1]
            del args[i:i + 2]
            return value
    return None


def need(args, count, usage):
    if len(args) < count:
        print("Usage: python git_sync.py " + usage)
        sys.exit(1)


def current_branch():
    return porcelain.active_branch(REPO).decode()


def cmd_clone(args):
    need(args, 1, "clone <url> [dir]")
    print(f"Cloning {args[0]}...")
    target = args[1] if len(args) > 1 else None
    porcelain.clone(args[0], target)
    print("Done!")


def cmd_init(args):
    porcelain.init(REPO)
    print("Initialized empty repository")


def cmd_status(args):
    st = porcelain.status(REPO)
    try:
        print(f"On branch {current_branch()}")
    except Exception:
        print("HEAD detached")
    staged = False
    for kind in ("add", "modify", "delete"):
        for path in st.staged[kind]:
            if not staged:
                print("\nChanges to be committed:")
                staged = True
            label = {"add": "new file", "modify": "modified", "delete": "deleted"}[kind]
            print(f"  {label}: {path.decode() if isinstance(path, bytes) else path}")
    if st.unstaged:
        print("\nChanges not staged for commit:")
        for path in st.unstaged:
            print(f"  modified: {path.decode() if isinstance(path, bytes) else path}")
    if st.untracked:
        print("\nUntracked files:")
        for path in st.untracked:
            print(f"  {path.decode() if isinstance(path, bytes) else path}")
    if not staged and not st.unstaged and not st.untracked:
        print("\nNothing to commit, working tree clean")


def cmd_add(args):
    need(args, 1, "add <paths...>")
    porcelain.add(REPO, args)


def cmd_commit(args):
    msg = option(args, "-m", "--message")
    do_all = flag(args, "-a", "--all")
    amend = flag(args, "--amend")
    if msg is None and not amend:
        print('Usage: python git_sync.py commit -m "message" [-a] [--amend]')
        sys.exit(1)
    sha = porcelain.commit(
        REPO,
        message=msg.encode("utf-8") if msg is not None else None,
        all=do_all,
        amend=amend,
    )
    print(f"Committed {sha.decode()[:7]}")


def cmd_push(args):
    msg = args[0] if args else "Update from school laptop"
    print("Adding and committing changes...")
    porcelain.add(REPO)
    porcelain.commit(REPO, message=msg.encode("utf-8"))
    print("Pushing to GitHub...")
    porcelain.push(REPO, "origin", "refs/heads/main")  # Change 'main' to 'master' if your repo uses master
    print("Pushed successfully!")


def cmd_publish(args):
    remote = args[0] if args else "origin"
    branch = args[1] if len(args) > 1 else current_branch()
    print(f"Pushing {branch} to {remote}...")
    porcelain.push(REPO, remote, "refs/heads/" + branch)
    print("Pushed successfully!")


def cmd_pull(args):
    print("Pulling latest code...")
    porcelain.pull(REPO, args[0] if args else "origin")
    print("Up to date!")


def cmd_fetch(args):
    porcelain.fetch(REPO, args[0] if args else "origin")
    print("Fetched")


def cmd_log(args):
    oneline = flag(args, "--oneline")
    stat = flag(args, "--stat")
    patch = flag(args, "-p", "--patch")
    name_only = flag(args, "--name-only")
    name_status = flag(args, "--name-status")
    reverse = flag(args, "--reverse")
    no_merges = flag(args, "--no-merges")
    count = option(args, "-n", "--max-count")
    author = option(args, "--author")
    grep = option(args, "--grep")
    since = option(args, "--since")
    until = option(args, "--until")
    if count is None:
        for a in list(args):
            if a.startswith("-") and a[1:].isdigit():
                count = a[1:]
                args.remove(a)
    if "--" in args:
        args.remove("--")
    porcelain.log(
        REPO,
        paths=args or None,
        max_entries=int(count) if count else None,
        reverse=reverse,
        name_only=name_only,
        name_status=name_status,
        author=author,
        grep=grep,
        since=since,
        until=until,
        no_merges=no_merges,
        oneline=oneline,
        stat=stat,
        patch=patch,
    )


def cmd_show(args):
    porcelain.show(REPO, args or None)


def cmd_diff(args):
    staged = flag(args, "--staged", "--cached")
    paths = None
    if "--" in args:
        i = args.index("--")
        paths = args[i + 1:] or None
        args = args[:i]
    commit = args[0] if len(args) > 0 else None
    commit2 = args[1] if len(args) > 1 else None
    sys.stdout.flush()
    porcelain.diff(REPO, commit=commit, commit2=commit2, staged=staged, paths=paths)
    sys.stdout.flush()


def cmd_ls_files(args):
    for path in porcelain.ls_files(REPO):
        print(path.decode())


def cmd_blame(args):
    need(args, 1, "blame <file>")
    for entry in porcelain.blame(REPO, args[0]):
        print(entry)


def cmd_rev_parse(args):
    need(args, 1, "rev-parse <name>")
    for name in args:
        obj = porcelain.rev_parse(REPO, name)
        obj = obj.id if hasattr(obj, "id") else obj
        print(obj.decode() if isinstance(obj, bytes) else obj)


def cmd_branch(args):
    delete = flag(args, "-d", "-D", "--delete")
    if delete:
        need(args, 1, "branch -d <name>")
        porcelain.branch_delete(REPO, args)
        print(f"Deleted branch {', '.join(args)}")
    elif args:
        porcelain.branch_create(REPO, args[0], args[1] if len(args) > 1 else None)
        print(f"Created branch {args[0]}")
    else:
        try:
            current = current_branch()
        except Exception:
            current = None
        for b in sorted(porcelain.branch_list(REPO)):
            name = b.decode() if isinstance(b, bytes) else b
            print(("* " if name == current else "  ") + name)


def cmd_checkout(args):
    new_branch = option(args, "-b")
    force = flag(args, "-f", "--force")
    if new_branch:
        porcelain.checkout(REPO, args[0] if args else None, force=force, new_branch=new_branch)
        print(f"Switched to new branch {new_branch}")
        return
    need(args, 1, "checkout <branch|commit> | checkout -b <name>")
    if "--" in args:
        i = args.index("--")
        porcelain.checkout(REPO, None, force=force, paths=args[i + 1:])
        return
    porcelain.checkout(REPO, args[0], force=force)
    print(f"Switched to {args[0]}")


def cmd_switch(args):
    create = option(args, "-c", "--create")
    force = flag(args, "-f", "--force")
    detach = flag(args, "--detach")
    if create:
        target = args[0] if args else "HEAD"
        porcelain.switch(REPO, target, create=create, force=force)
        print(f"Switched to new branch {create}")
        return
    need(args, 1, "switch <branch> | switch -c <name>")
    porcelain.switch(REPO, args[0], force=force, detach=detach)
    print(f"Switched to {args[0]}")


def cmd_merge(args):
    no_ff = flag(args, "--no-ff")
    need(args, 1, "merge <branch> [--no-ff]")
    sha, conflicts = porcelain.merge(REPO, args[0], no_ff=no_ff)
    if conflicts:
        print("Merge conflicts in:")
        for c in conflicts:
            print("  " + c.decode())
    elif sha:
        print(f"Merged, new commit {sha.decode()[:7]}")
    else:
        print("Merge complete")


def cmd_tag(args):
    delete = flag(args, "-d", "--delete")
    msg = option(args, "-m", "--message")
    if delete:
        need(args, 1, "tag -d <name>")
        porcelain.tag_delete(REPO, args[0])
        print(f"Deleted tag {args[0]}")
    elif args:
        porcelain.tag_create(
            REPO,
            args[0],
            message=msg,
            annotated=msg is not None,
            objectish=args[1] if len(args) > 1 else "HEAD",
        )
        print(f"Created tag {args[0]}")
    else:
        for t in sorted(porcelain.tag_list(REPO)):
            print(t.decode() if isinstance(t, bytes) else t)


def cmd_reset(args):
    mode = "mixed"
    for m in ("soft", "mixed", "hard"):
        if flag(args, "--" + m):
            mode = m
    porcelain.reset(REPO, mode, args[0] if args else "HEAD")
    print(f"Reset ({mode}) to {args[0] if args else 'HEAD'}")


def cmd_restore(args):
    staged = flag(args, "--staged")
    source = option(args, "--source", "-s")
    need(args, 1, "restore <paths...> [--staged] [--source=<commit>]")
    porcelain.restore(REPO, args, source=source, staged=staged, worktree=not staged)


def cmd_rm(args):
    cached = flag(args, "--cached")
    need(args, 1, "rm <paths...> [--cached]")
    porcelain.rm(REPO, args, cached=cached)


def cmd_mv(args):
    force = flag(args, "-f", "--force")
    need(args, 2, "mv <src> <dst>")
    porcelain.mv(REPO, args[0], args[1], force=force)


def cmd_revert(args):
    need(args, 1, "revert <commit>")
    porcelain.revert(REPO, args[0])
    print("Reverted")


def cmd_cherry_pick(args):
    need(args, 1, "cherry-pick <commit>")
    porcelain.cherry_pick(REPO, args[0])
    print("Cherry-picked")


def cmd_clean(args):
    porcelain.clean(REPO, REPO)
    print("Removed untracked files")


def cmd_stash(args):
    sub = args[0] if args else "push"
    if sub in ("push", "save"):
        porcelain.stash_push(REPO)
        print("Saved working directory to stash")
    elif sub == "pop":
        porcelain.stash_pop(REPO)
        print("Stash applied")
    elif sub == "list":
        for index, entry in porcelain.stash_list(REPO):
            print(f"stash@{{{index}}}: {entry[1].decode(errors='replace') if isinstance(entry, tuple) else entry}")
    else:
        print("Usage: python git_sync.py stash [push|pop|list]")


def cmd_remote(args):
    if not args or args[0] in ("-v", "--verbose"):
        verbose = bool(args)
        from dulwich.repo import Repo
        config = Repo(REPO).get_config()
        for section in config.sections():
            if section[0] == b"remote":
                name = section[1].decode()
                if verbose:
                    url = config.get(section, b"url").decode()
                    print(f"{name}\t{url}")
                else:
                    print(name)
    elif args[0] == "add":
        need(args, 3, "remote add <name> <url>")
        porcelain.remote_add(REPO, args[1], args[2])
        print(f"Added remote {args[1]}")
    elif args[0] in ("remove", "rm"):
        need(args, 2, "remote remove <name>")
        from dulwich.repo import Repo
        porcelain.remote_remove(Repo(REPO), args[1])
        print(f"Removed remote {args[1]}")
    else:
        print("Usage: python git_sync.py remote [-v] | add <name> <url> | remove <name>")


COMMANDS = {
    "clone": cmd_clone,
    "init": cmd_init,
    "status": cmd_status,
    "add": cmd_add,
    "commit": cmd_commit,
    "push": cmd_push,
    "publish": cmd_publish,
    "pull": cmd_pull,
    "fetch": cmd_fetch,
    "log": cmd_log,
    "show": cmd_show,
    "diff": cmd_diff,
    "ls-files": cmd_ls_files,
    "blame": cmd_blame,
    "rev-parse": cmd_rev_parse,
    "branch": cmd_branch,
    "checkout": cmd_checkout,
    "switch": cmd_switch,
    "merge": cmd_merge,
    "tag": cmd_tag,
    "reset": cmd_reset,
    "restore": cmd_restore,
    "rm": cmd_rm,
    "mv": cmd_mv,
    "revert": cmd_revert,
    "cherry-pick": cmd_cherry_pick,
    "clean": cmd_clean,
    "stash": cmd_stash,
    "remote": cmd_remote,
}

action = sys.argv[1].lower() if len(sys.argv) > 1 else "help"
rest = sys.argv[2:]

if action in COMMANDS:
    COMMANDS[action](rest)
else:
    print(HELP)
