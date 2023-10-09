---
title: 搭建自己的第一篇博客
description: 利用caddy v2 + webhook + hugo搭建博客, 并配置bitwarden与v2ray服务
date: 2023-03-16T02:02:45+08:00
categories:
  - 服务器
keywords:
  - caddy
  - webhook
  - hugo
  - v2ray
  - bitwarden
tags:
  - caddy
  - webhook
  - hugo
  - v2ray
  - bitwarden
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/84738802_p0_master1200.jpg
toc: true
comments: true
hidden: false
draft: false
---

# Caddy v2, webhook, hugo搭建博客并配置bitwarden与v2ray

## 项目概述

### 版本说明

#### caddy

之前我的服务器(`amd64`, `debain10`)上一直用的是`caddy 1.x`版本, 目前官方为了推广2.0版本, 已经绝情的将v1版本的官方下载渠道, 插件安装脚本以及文档等全都删掉了🙃, 导致我最开始看的关于v1版搭建博客的教程全都用不了. 并且由于v1版已经发布很多年了, 而v2在2020年才发布. 网上绝大部分教程以及`git`, `webhook`相关的插件还都是v1版的, 对纯萌新来说挺难找到合适的教程来详细描述具体如何配置. 目前网上也有了很多大佬开源的包含有`caddy v2`, `hugo`等的`docker`镜像可以方便的一键搭建博客. 但不知道为什么在我的服务器上运行时似乎没有办法正常监听端口一样，总是接受不到相关的请求

此外, `caddy` v2与v1相比配置文件改成了原生`json`格式, 之前v1版本的时候使用的配置文件是`Caddyfile`, 写起来即简便又易读. 现在换成`json`之后, 虽然还是可以用`Caddyfile`来写, 但是实际运行`caddy`似乎会先将`Caddyfile`转化成`json`然后再运行. 这就导致写在`Caddyfile`里的配置, 尤其是相关插件的配置很可能在转化的时候出现某些问题导致运行失败. 还有另外一个很重要的问题是, 大多v2版本的`caddy`插件是支持`Caddyfile`的写法的, 但有些插件就只有`json`格式的配置写法. 而前面也提到`Caddyfile`是可以被`caddy`转化为`json`格式的, 可以使用`caddy adapt`命令显式转化文件。所以即使部分插件在文档中只写了`Caddyfile`的写法, 你也可以先写这部分的配置写道`Caddyfile`里然后再转化为`json`继续配置

总上所述, 最终选择了本地使用`xcaddy`构建带有`vrongmeal/caddygit`, `WingLim/caddy-webhook `, `mholt/caddy-webdav`这三个插件的`caddy v2.6.4`在本地运行`json`格式配置文件的方式完成服务器的路由配置

#### `bitwarden`

`bitwarden`是一个开源, 方便的密码管理器, 在网页(`chrome`, `firefox`, `safair`)/`android`/`ios`/`windows`/`linux`等全平台都有相关客户端(网页插件), 支持部署到自己的服务器构建自己专属的密码库，有效的提高了安全性. 之前`caddy`配置`bitwarden`的时候, 可以使用官方的docker镜像, 只用映射一下端口就可以一键部署. 现在`caddy`升级到v2之后好像对`bitwarden`还不适配, 只能使用最新版本的`vaultwarden`(`bitwarden`的一个第三方版本)来部署

#### `hugo`

`hugo`是一个方便的从md文档构建静态网页的工具, 非常符合我目前的需求. 但是注意很多网站的主题要求`hugo`是`extend`版(`hugo`官方也推荐使用`extend`版), 同时由于主题模板的更新, 很多主题对`hugo`的版本要求会比较新. 但`apt`安装的是0.55版的, 不符合我目前主题的要求. 官方还提供了snap安装, 但我在服务器上装的时候出现了好像说系统不匹配的错误😔, 看github的issue上有人提过类似的, 下面有个回答说他已经修复了, 可是我的这个还是不行. 所以最后采用了在github的release里下载`extend`版的deb安装包，然后再在本地`dpkg -i`安装

### 格式说明

本文假定你已经有一个属于自己的服务器和域名, 并且已经把dns解析好了, 国内服务器的备案也不在本文讨论范围内. 本文仅包含`caddy`的安装配置, 配置文件`caddyfile.json`的编写, `hugo`的安装配置和`vaultwarden` docker镜像的配置

本文接下来在配置文件中出现的必需需要读者本人根据自己服务器等进行修改的地方会使用类似`@1{path}`的方式代指，例如你本地的某个文件夹地址为`/usr/local/bin`, 在配置文件中会以`@1{path}`代指这个地址(数字指的是这个文件中出现的第几个, 如果两个地方用一个的话就是一个地址). 同理`@1{domain}`代指文件中第一个出现的需要你修改成自己的域名`www.your-domain.com`

## 安装

### caddy

`caddy`从官方途径`apt`安装的是没有插件的, 没插件没办法完成接下来的部署, 但是从`apt`安装的会配套相关的systemd服务, 方便后续管理运行重启等. 因此先`apt`安装没有插件的`caddy`然后再通过`xcaddy`来编译带有插件的`caddy`, 编译完成后替换掉之前`apt`安装的`caddy`

#### apt安装caddy

```bash
apt install -y debian-keyring debian-archive-keyring apt-transport-https
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' | gpg --dearmor -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' | tee /etc/apt/sources.list.d/caddy-stable.list
apt update
apt install caddy
```

#### go环境

由于`xcaddy`是由`go`语言编译安装的, 所以需要先配置环境

```bash
wget https://go.dev/dl/go1.20.7.linux-amd64.tar.gz
rm -rf /usr/local/go && tar -C /usr/local -xzf go1.20.7.linux-amd64.tar.gz && rm go1.20.7.linux-amd64.tar.gz
vim ~/.bashrc

#写入文件
export PATH=$PATH:/usr/local/go/bin
#保存文件

source ~/.bashrc
```

#### xcaddy

安装`xcaddy`

```bash
go install github.com/caddyserver/xcaddy/cmd/xcaddy@latest
```

`xcaddy`编译带插件的`caddy`

```bash
export version=$(curl -s "https://api.github.com/repos/caddyserver/caddy/releases/latest" | grep tag_name | cut -f4 -d "\"")
go/bin/xcaddy build ${version} \
	--with github.com/WingLim/caddy-webhook \
    --with github.com/mholt/caddy-webdav \
    --with github.com/vrongmeal/caddygit
```

之后会编译出带有这三个插件的`caddy`到当前目录, 如果你需要其他插件的话, 后面`--with`接着跟插件的`github`仓库地址就行了

编译完成后使用`whereis caddy`查看之前`apt`安装的`caddy`安装到哪里了, 再用`mv`把刚编译的带插件的替换掉`apt`安装的

```bash
mv caddy /usr/bin/caddy
```

### hugo

```bash
export version=$(curl -s "https://api.github.com/repos/gohugoio/hugo/releases/latest" | grep tag_name | cut -f4 -d "\"" | grep -oE '([0-9]+\.?)+')
wget https://github.com/gohugoio/hugo/releases/download/v${version}/hugo_extended_${version}_linux-amd64.deb
dpkg -i hugo_extended_${version}_linux-amd64.deb
```

### vaultwarden {#vaultwarden_setup}

需要先安装`docker`, 安装好后再

```bash
apt-get install ca-certificates curl gnupg
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
apt-get update
apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin


docker run -d --name vaultwarden \
  -e SIGNUPS_ALLOWED=true \
  -e WEBSOCKET_ENABLED=true \
  -e LOG_FILE=/data/bitwarden.log \
  -p @1{vault_port}:80 \
  -p @2{vault_port}:3012 \
  -v /vw-data/:/data/ \
  vaultwarden/server:1.27.0
```

此处的`@1{vault-port}`和`@2{vault-port}`需要替换掉成之后你在`caddy`的配置文件中反代的端口, 因此建议配置好其他之后再安装这个

## 配置

变量解释如下，需要将下面所有配置文件中出现的地方替换成你自己的

```
@1{vault_port}:80	--caddy反代的bitwarden端口，例如8080：80
@2{vault_port}:3012	--caddy反代的bitwarden端口，例如3012：3012
@3{v2ray_port}	--caddy反代的v2ray端口，例如2233

@1{cer_path}	--bitwarden网站的证书cer部分，例如usr/local/etc/ssl/bitwarden.cer
@1{key_path} 	--bitwarden网站的证书key部分, 例如usr/local/etc/ssl/bitwarden.key
@2{cer_path}	--blog网站的证书cer部分
@2{key_path}	--blog网站的证书key部分
@3{v2ray_path}	--v2ray的访问路径，以/开头，例如/vvv。但是不能和@4{v2ray_www_path}的最后一级相同，即如果@4{v2ray_www_path}是/var/www/v2ray的话，就不能是/v2ray
@4{v2ray_www_path}	--v2ray网站的存放路径，例如/var/www/v2ray
@5{blog_path}	--存放博客网站的地址, 例如/var/www/myblog

@1{vault_domain}	--bitwarden的域名
@2{blog_domain}		--blog的域名
@3{v2ray_domain}	--v2ray的伪装域名

@1{github_url}		--存放博客文档的github仓库地址
@1{github_token}	--有存放博客文档的github仓库访问权限的token
@1{github_secret}	--webhook的secret

@1{v2ray_uuid}	--v2ray配置
@1{your_email}	--注册域名时使用的邮箱

@1{cloudflare_key}	--cloudflare的key
```

### caddy

#### caddy.service

首先需要修改以下`caddy.service`配置相关服务

```bash
vim /etc/systemd/system/caddy.service


# caddy.service
#
# For using Caddy with a config file.
#
# Make sure the ExecStart and ExecReload commands are correct
# for your installation.
#
# See https://caddyserver.com/docs/install for instructions.
#
# WARNING: This service does not use the --resume flag, so if you
# use the API to make changes, they will be overwritten by the
# Caddyfile next time the service is restarted. If you intend to
# use Caddy's API to configure it, add the --resume flag to the
# `caddy run` command or use the caddy-api.service file instead.

[Unit]
Description=Caddy
Documentation=https://caddyserver.com/docs/
After=network.target network-online.target
Requires=network-online.target

[Service]
Type=notify
User=caddy
Group=caddy
ExecStart=/usr/bin/caddy run --environ --config /etc/caddy/caddyfile.json
ExecReload=/usr/bin/caddy reload --config /etc/caddy/caddyfile.json --force
TimeoutStopSec=5s
LimitNOFILE=1048576
LimitNPROC=512
PrivateTmp=true
ProtectSystem=full
AmbientCapabilities=CAP_NET_BIND_SERVICE

[Install]
WantedBy=multi-user.target


chown root:root /etc/systemd/system/caddy.service
chmod 644 /etc/systemd/system/caddy.service
systemctl daemon-reload

chown root:root /usr/bin/caddy
chmod 755 /usr/bin/caddy
setcap 'cap_net_bind_service=+ep' /usr/bin/caddy
groupadd -g 33 caddy
useradd \
  -g caddy --no-user-group \
  --home-dir /var/caddy --no-create-home \
  --shell /usr/sbin/nologin \
  --system --uid 33 caddy

mkdir /etc/caddy
mkdir /var/www
mkdir /var/caddy
mkdir /etc/ssl/caddy
touch /etc/caddy/caddyfile.json
touch /var/log/caddy.log
chown caddy:caddy /var/caddy
chown -R root:caddy /etc/caddy
chown -R root:caddy /etc/ssl/caddy
chown -R root:caddy /var/www
chmod 555 /var/caddy
chmod 777 /var/www
chmod 0770 /etc/ssl/caddy
chown root:caddy /var/log/caddy.log
chown root:root /etc/caddy/caddyfile.json
chmod 770 /var/log/caddy.log
chmod 644 /etc/caddy/caddyfile.json
```

#### caddyfile.json

接着配置`caddyfile.json`

```json
vim /etc/caddy/caddyfile.json

{
  "logging": {
    "logs": {
      "default": {
        "exclude": [
          "http.log.access.log0",
          "http.log.access.log1",
          "http.log.access.log2"
        ]
      },
      "log0": {
        "writer": {
          "filename": "/var/log/caddy.log",
          "output": "file",
          "roll_keep": 10,
          "roll_size_mb": 10
        },
        "level": "INFO",
        "include": [
          "http.log.access.log0"
        ]
      },
      "log1": {
        "writer": {
          "filename": "/var/log/caddy.log",
          "output": "file"
        },
        "include": [
          "http.log.access.log1"
        ]
      },
      "log2": {
        "writer": {
          "filename": "/var/log/caddy.log",
          "output": "file"
        },
        "include": [
          "http.log.access.log2"
        ]
      }
    }
  },
  "apps": {
    "tls": {
      "certificates": {
        "load_files": [
          {
            "certificate": "@1{cer_path}",
            "key": "@1{key_path}",
            "tags": [
              "cert0"
            ]
          },
          {
            "certificate": "@2{cer_path}",
            "key": "@2{cer_path}",
            "tags": [
              "cert1"
            ]
          }
        ]
      },
      "automation": {
        "policies": [
          {
            "subjects": [
              "@1{vault_domain}",
              "@2{blog_domain}"
            ]
          },
          {
            "subjects": [
              "@3{v2ray_domain}"
            ],
            "issuers": [
              {
                "email": "@1{your_email}",
                "module": "acme"
              },
              {
                "email": "@1{your_email}",
                "module": "zerossl"
              }
            ]
          }
        ]
      }
    },
    "http": {
      "servers": {
        "srv1": {
          "listen": [
            ":80"
          ],
          "routes": [
            {
              "match": [
                {
                  "host": [
                    "@1{vault_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "handler": "static_response",
                          "headers": {
                            "Location": [
                              "https://@1{vault_domain}"
                            ]
                          },
                          "status_code": 302
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true
            },
            {
              "match": [
                {
                  "host": [
                    "@2{blog_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "handler": "static_response",
                          "headers": {
                            "Location": [
                              "https://@2{blog_domain}"
                            ]
                          },
                          "status_code": 302
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true
            },
            {
              "match": [
                {
                  "host": [
                    "@3{v2ray_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "handler": "static_response",
                          "headers": {
                            "Location": [
                              "https://@3{v2ray_domain}"
                            ]
                          },
                          "status_code": 302
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true
            }
          ]
        },
        "srv0": {
          "listen": [
            ":443"
          ],
          "tls_connection_policies": [
            {
              "match": {
                "sni": [
                  "@2{blog_domain}"
                ]
              },
              "certificate_selection": {
                "any_tag": [
                  "cert1"
                ]
              }
            },
            {
              "match": {
                "sni": [
                  "@1{vault_domain}"
                ]
              },
              "certificate_selection": {
                "any_tag": [
                  "cert0"
                ]
              }
            },
            {
              "match": {
                "sni": [
                  "@3{v2ray_domain}"
                ]
              },
              "cipher_suites": [
                "TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384",
                "TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305_SHA256"
              ],
              "protocol_min": "tls1.2",
              "protocol_max": "tls1.3"
            },
            {}
          ],
          "logs": {
            "logger_names": {
              "@3{v2ray_domain}": "log0"
            },
            "logger_names": {
              "@1{vault_domain}": "log1"
            },
            "logger_names": {
              "@2{blog_domain}": "log2"
            },
            "skip_hosts": [
              "@1{vault_domain}",
              "@2{blog_domain}",
              "@3{v2ray_domain}"
            ]
          },
          "routes": [
            {
              "match": [
                {
                  "host": [
                    "@3{v2ray_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "handler": "vars",
                          "root": "@4{v2ray_www_path}"
                        },
                        {
                          "encodings": {
                            "gzip": {}
                          },
                          "handler": "encode",
                          "prefer": [
                            "gzip"
                          ]
                        }
                      ]
                    },
                    {
                      "handle": [
                        {
                          "handler": "reverse_proxy",
                          "upstreams": [
                            {
                              "dial": "localhost:@3{v2ray_port}"
                            }
                          ]
                        }
                      ],
                      "match": [
                        {
                          "header": {
                            "Connection": [
                              "*Upgrade*"
                            ],
                            "Upgrade": [
                              "websocket"
                            ]
                          },
                          "path": [
                            "@3{v2ray_path}"
                          ]
                        }
                      ]
                    },
                    {
                      "handle": [
                        {
                          "handler": "file_server",
                          "hide": [
                            "/etc/caddy/Caddyfile"
                          ]
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true
            },
            {
              "match": [
                {
                  "host": [
                    "@1{vault_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "encodings": {
                            "gzip": {}
                          },
                          "handler": "encode",
                          "prefer": [
                            "gzip"
                          ]
                        }
                      ]
                    },
                    {
                      "handle": [
                        {
                          "handler": "reverse_proxy",
                          "upstreams": [
                            {
                              "dial": "localhost:@2{vault_port}"
                            }
                          ]
                        }
                      ],
                      "match": [
                        {
                          "path": [
                            "/notifications/hub"
                          ]
                        }
                      ]
                    },
                    {
                      "handle": [
                        {
                          "handler": "reverse_proxy",
                          "headers": {
                            "request": {
                              "set": {
                                "X-Real-Ip": [
                                  "localhost"
                                ]
                              }
                            }
                          },
                          "upstreams": [
                            {
                              "dial": "localhost:@1{vault_port}"
                            }
                          ]
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true
            },
            {
              "match": [
                {
                  "host": [
                    "@5{blog_domain}"
                  ]
                }
              ],
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "handler": "vars",
                          "root": "@5{blog_path}/public"
                        },
                        {
                          "handler": "file_server",
                          "hide": [
                            "/etc/caddy/Caddyfile"
                          ]
                        }
                      ]
                    }
                  ]
                }
              ],
              "terminal": true,
              "match": [
                {
                  "not": [
                    {
                      "path": [
                        "/webhook"
                      ]
                    }
                  ]
                }
              ]
            },
            {
              "handle": [
                {
                  "handler": "subroute",
                  "routes": [
                    {
                      "handle": [
                        {
                          "branch": "main",
                          "handler": "webhook",
                          "path": "@5{blog_path}/content",
                          "repo": "@1{github_url}",
                          "token": "@1{github_token}",
                          "secret": "@1{github_secret}",
                          "command": [
                            "hugo",
                            "--destination",
                            "@5{blog_path}/public"
                          ],
                          "submodule": true,
                          "type": "github"
                        }
                      ]
                    }
                  ]
                }
              ],
              "match": [
                {
                  "path": [
                    "/webhook"
                  ]
                }
              ]
            }
          ]
        }
      }
    }
  }
}

```

### v2ray

https://github.com/zxcvos/Xray-script

#### config.json

mkdir -p /usr/local/etc/v2ray/

mkdir -p /usr/local/etc/ssl/ecc/

vim /usr/local/etc/v2ray/config.json

```json
{
  "log": {
    "access": "/var/log/v2ray/access.log",
    "error": "/var/log/v2ray/error.log",
    "loglevel": "warning"
  },
  "dns": {},
  "stats": {},
  "inbounds": [
    {
      "port": @3{v2ray_port},
      "listen": "127.0.0.1",
      "protocol": "vmess",
      "settings": {
        "clients": [
          {
            "id": "@1{v2ray_uuid}",
            "level": 0,
            "email": "@1{your_email}"
          }
        ],
        "decryption": "none"
      },
      "streamSettings": {
        "network": "ws",
        "security": "none",
        "wsSettings": {
          "path": "@3{v2ray_path}"
        }
      }
    }
  ],
  "outbounds": [
    {
      "tag": "direct",
      "protocol": "freedom",
      "settings": {}
    },
    {
      "tag": "blocked",
      "protocol": "blackhole",
      "settings": {}
    }
  ],
  "routing": {
    "domainStrategy": "IPIfNonMatch",
    "rules": [
      {
        "type": "field",
        "domain": [
          "googleapis.cn",
          "goofle.cn"
        ],
        "outboundTag": "direct"
      },
      {
        "outboundTag": "blocked",
        "ip": [
          "geoip:private"
        ],
        "type": "field"
      }
    ]
  },
  "policy": {},
  "reverse": {},
  "transport": {}
}

```

### hugo

```bash
hugo new site @5{blog_path}
```

会在`@5{blog_path}`处建立网站的目录

### [vaultwarden](#vaultwarden_setup)

### 编写脚本监视git活动并更新网页

按理来说是不需要这一步的，应该是`caddy`接收到`github`发来的`webhook`然后就启动相关插件把更新的仓库`pull`下来, 然后就调用`hugo --destination @5{blog_path}/public`命令更新网页了

但实际上发现这个命令居然是在`/`根目录下执行的, 即使在`caddyfile.json`里指定`root * @5{blog_path} `了也不行.

开始还想把命令换成执行其他的脚本, 然后在脚本中`cd`到路径再执行`hugo`生成网页. 但是好像也没法正常运行起来

只能退而求其次, 单独编写一个`inotify.sh`监测`@5{blog_path}/content/.git/index`这个文件的变动情况然后再执行更新网页, 脚本如下(需要先安装`inotify`)

```bash
#!/bin/sh
filename=$@5{blog_path}/content/.git/index
inotifywait -mrq --format '%e' --event create,delete,modify  $filename | while read event
  do
	cd @5{blog_path} && hugo
  done
```

之后执行`nohup bash inotify.sh > /dev/null 2>&1`让脚本在后台运行就可以了

## 为网站申请证书 (以cloudflare为例)

安装`acme.sh`

```bash
curl https://get.acme.sh | sh
source ~/.bashrc
```

```bash
export CF_Key="@1{cloudflare_key}"
export CF_Email="@1{your_email}"
acme.sh --register-account -m @1{your_email}


```

之后运行如下命令即可为`blog`网站申请证书

```bash
acme.sh --issue --dns dns_cf -d @2{blog_domain} --keylength ec-256 #--debug 2 如果出错可以加这个参数看看哪里报错
```

如下命令可将证书安装在`@2{cer_path}`, `@2{key_path}`路径下

```bash
acme.sh --installcert -d @2{blog_domain} --ecc --key-file @2{key_path} --fullchain-file @2{cer_path} --force
```



```bash
vim /etc/sysctl.conf

# max open files
fs.file-max = 51200
# max read buffer
net.core.rmem_max = 67108864
# max write buffer
net.core.wmem_max = 67108864
# default read buffer
net.core.rmem_default = 65536
# default write buffer
net.core.wmem_default = 65536
# max processor input queue
net.core.netdev_max_backlog = 4096
# max backlog
net.core.somaxconn = 4096
# resist SYN flood attacks
net.ipv4.tcp_syncookies = 1
# reuse timewait sockets when safe
net.ipv4.tcp_tw_reuse = 1
# turn off fast timewait sockets recycling
net.ipv4.tcp_tw_recycle = 0
# short FIN timeout
net.ipv4.tcp_fin_timeout = 30
# short keepalive time
net.ipv4.tcp_keepalive_time = 1200
# outbound port range
net.ipv4.ip_local_port_range = 10000 65000
# max SYN backlog
net.ipv4.tcp_max_syn_backlog = 4096
# max timewait sockets held by system simultaneously
net.ipv4.tcp_max_tw_buckets = 5000
# TCP receive buffer
net.ipv4.tcp_rmem = 4096 87380 67108864
# TCP write buffer
net.ipv4.tcp_wmem = 4096 65536 67108864
# turn on path MTU discovery
net.ipv4.tcp_mtu_probing = 1
# for high-latency network
net.core.default_qdisc=fq
net.ipv4.tcp_congestion_control = bbr

sysctl -p

apt install libcap2-bin
setcap 'cap_net_bind_service=+ep' /usr/bin/caddy

chown root:root /usr/bin/caddy
chown -R root:root /etc/caddy
chown -R root:www-data /etc/ssl/caddy
chown root:www-data /var/log/caddy.log
chown root:root /etc/systemd/system/caddy.service

chmod 755 /usr/bin/caddy
chmod 770 /etc/ssl/caddy
chmod 770 /var/log/caddy.log
chmod 644 /etc/systemd/system/caddy.service


chown root:root /usr/local/etc/v2ray/v2ray.cer
chown root:root /usr/local/etc/v2ray/v2ray.key
chmod 644 /usr/local/etc/v2ray/v2ray.cer
chmod 644 /usr/local/etc/v2ray/v2ray.key

chown root:root /usr/local/etc/ssl/ecc/password.cer
chown root:root /usr/local/etc/ssl/ecc/password.key
chmod 644 /usr/local/etc/ssl/ecc/password.cer
chmod 644 /usr/local/etc/ssl/ecc/password.key

chown root:root /usr/local/etc/v2ray/txt.cer
chown root:root /usr/local/etc/v2ray/txt.key
chmod 644 /usr/local/etc/v2ray/txt.cer
chmod 644 /usr/local/etc/v2ray/txt.key

mkdir /var/www/public
vim /var/www/public/index.html
```



## 常用命令表

```bash
bash <(curl -L -s check.unlock.media)

acme:
    export CF_Key=""
    export CF_Email=""
    acme.sh --register-account -m example@gmail.com
    acme.sh --issue --dns dns_cf -d example.com --keylength ec-256 --debug 2
    acme.sh --installcert -d example.com --ecc --key-file /usr/local/etc/ssl/example.key --fullchain-file /usr/local/etc/ssl/example.cer --force

caddy:
	vim /etc/caddy/caddyfile.json
    systemctl daemon-reload
    caddy list-modules	#查看caddy安装了哪些插件
    journalctl --boot -u caddy.service	#查看caddy详细运行日志
    systemctl restart caddy && systemctl status caddy	#重启caddy并查看运行状态

v2ray：
    vim /usr/local/etc/v2ray/config.json
	systemctl daemon-reload
	systemctl stop v2ray
    systemctl restart v2ray && systemctl status v2ray
    journalctl -u v2ray
	bash install-release.sh	#更新v2ray版本

ufw:
	utf allow 80	#防火墙开启80端口允许外界访问， 需要提前apt安装ufw
	ufw delete 80	#关闭80端口
    ufw status numbered	#查看开启了哪些端口

docker：
	docker run -d --name vaultwarden \
  -e SIGNUPS_ALLOWED=true \
  -e WEBSOCKET_ENABLED=true \
  -e LOG_FILE=/data/bitwarden.log \
  -p @1{vault_port}:80 \
  -p @2{vault_port}:3012 \
  -v /vw-data/:/data/ \
  vaultwarden/server:1.27.0

xcaddy：
	xcaddy build v2.6.0 \
	--with github.com/WingLim/caddy-webhook \
	--with github.com/mholt/caddy-webdav \
	--with github.com/vrongmeal/caddygit

inotify:
	  nohup bash inotify.sh > /dev/null 2>&1
```

## 其他

搭建所用的所有配置文件如上所示, 接下来写写我在搭建中遇到的一些问题

1.   原本v2ray使用的是h2+tls协议, 但是似乎没法在caddy v2中正常运行, 然后看到好多教程都是用ws, 我就也改成了ws, 然后就顺利运行了
2.   bitwarden之前的官方docker镜像是可以直接监听443端口传来的https请求. 但新版的Vaultwarden要配备https的话好像很麻烦, 并且申请的证书是在docker容器里的, caddy无法访问. 但实际上可以先给域名申请证书, https的请求发给caddy之后再让caddy转发到aultwarden的docker容器映射的80端口那里. 这样实际访问的时候依然是https加密访问. 不用担心安全问题

## 参考资料

在搭建这篇博客时参考了以下链接, 感谢各位大佬的分享

### `caddy`:

[Caddy2 简明教程](https://mritd.com/2021/01/07/lets-start-using-caddy2)

[利用Caddy实现Hugo个人博客的自动化部署](https://blog.wangjunfeng.com/post/caddy-hugo/#%e6%b3%a8%e5%86%8ccaddy%e6%9c%8d%e5%8a%a1)

[vrongmeal/caddygit](https://github.com/LadderOperator/docker-caddy2-hugo-alidns)

[WingLim/caddy-webhook](https://github.com/vrongmeal/caddygit)

[WingLim/caddy-webhook](https://github.com/WingLim/caddy-webhook)

[LadderOperator/docker-caddy2-hugo-alidns](https://github.com/LadderOperator/docker-caddy2-hugo-alidns)

[caddy community](https://caddy.community/t/caddy-reverse-proxy-nextcloud-collabora-vaultwarden-with-local-https/12052/2)

### `vaultwarden`:

[dani-garcia/vaultwarden](https://github.com/dani-garcia/vaultwarden/wiki/Proxy-examples)

### `hugo`:

[hugo-command-usage](https://www.andbible.com/post/hugo-command-usage)

[Hugo 目录组织](https://www.jianshu.com/p/c5297a8bb1e7)

[Hugo 静态博客食用指南](https://niceram.xyz/2021/03/04/20210304_1125)

[Stack主题](https://site.zhelper.net/Hugo/hugo-stack)
