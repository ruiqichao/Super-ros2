#!/bin/bash

# SSH密钥自动化检查和Git操作测试脚本

echo "🔍 检查SSH密钥状态..."

# 1. 密钥存在性检查
check_ssh_key() {
    if [ -f "$HOME/.ssh/id_ed25519" ] || [ -f "$HOME/.ssh/id_rsa" ]; then
        echo "✅ SSH密钥已存在"
        if [ -f "$HOME/.ssh/id_ed25519" ]; then
            echo "🔑 使用ED25519密钥: $HOME/.ssh/id_ed25519"
        else
            echo "🔑 使用RSA密钥: $HOME/.ssh/id_rsa"
        fi
        return 0
    else
        echo "❌ 未找到SSH密钥"
        return 1
    fi
}

# 2. 密钥自动生成
generate_ssh_key() {
    echo "🔧 正在生成新的SSH密钥..."
    ssh-keygen -t ed25519 -C "$(git config user.email)" -f "$HOME/.ssh/id_ed25519" -N ""
    echo "✅ SSH密钥生成完成"
    echo ""
    echo "📋 请将以下公钥添加到您的GitHub账户："
    echo "=================================================="
    cat "$HOME/.ssh/id_ed25519.pub"
    echo "=================================================="
    echo ""
    echo "🔗 添加位置：https://github.com/settings/keys"
    echo "⚠️  添加完成后请按回车继续..."
    read -p "按回车键继续: " 
}

# 3. 仓库URL转换
convert_to_ssh_url() {
    local https_url=$1
    # 将HTTPS URL转换为SSH格式
    echo "$https_url" | sed 's/https:\/\/github.com\//git@github.com:/' | sed 's/\.git$/.git/'
}

# 4. 完整的Git推送流程
git_push_with_ssh() {
    local repo_url=$1
    local branch=${2:-master}
    
    echo "🚀 开始Git推送流程..."
    
    # 1. 检查SSH密钥
    if ! check_ssh_key; then
        generate_ssh_key
    fi
    
    # 2. 转换为SSH URL（如果需要）
    if [[ $repo_url == https://* ]]; then
        echo "🔄 转换仓库URL为SSH格式..."
        repo_url=$(convert_to_ssh_url "$repo_url")
        echo "新URL: $repo_url"
        git remote set-url origin "$repo_url"
    fi
    
    # 3. 执行Git操作
    echo "📝 添加文件到暂存区..."
    git add .
    
    echo "💾 创建提交..."
    git commit -m "Auto-commit: $(date '+%Y-%m-%d %H:%M:%S')"
    
    echo "📤 推送到远程仓库..."
    git push origin "$branch" --force
    
    echo "🎉 Git推送完成！"
}

# 测试执行
echo "🧪 测试SSH密钥检查功能..."
check_ssh_key

echo ""
echo "📊 当前Git状态:"
git status --short

echo ""
echo "🔗 当前远程仓库:"
git remote -v

# 如果需要测试完整的推送流程，取消下面的注释
# git_push_with_ssh "https://github.com/ruiqichao/Super-ros2.git" "master"