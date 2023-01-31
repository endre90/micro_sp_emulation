// use std::fs::File;
// use std::io::Write;
// use quote::quote;
// use syn::{parse_str, DeriveInput};

// mod generated;

// fn main() {
//     let message_definition = r#"
//         struct MyMessage {
//             Int32: data
//         }
//     "#;

//     let ast: DeriveInput = parse_str(message_definition).unwrap();
//     let name = &ast.ident;
//     let fields: Vec<syn::Field> = match &ast.data {
//         syn::Data::Struct(s) => s.fields.iter().cloned().collect(),
//         _ => panic!("expected a struct"),
//     };

//     let field_defs = fields.iter().map(|f| {
//         let ident = &f.ident;
//         let ty = &f.ty;
//         quote! {
//             pub #ident: #ty,
//         }
//     });

//     let expanded = quote! {
//         #[derive(Debug,Clone, PartialEq)]
//         pub struct #name {
//             #(#field_defs)*
//         }
//     };

//     let mut file = File::create("src/generated.rs").unwrap();
//     file.write_all(expanded.to_string().as_bytes()).unwrap();
//     println!("cargo:rerun-if-changed=build.rs");
// }

// use std::fs::File;
// use std::io::Write;
// use quote::quote;

// fn main() {
//     let mut file = File::create("src/MyMessage.msg").unwrap();
//     let struct_name = "MyStruct";
//     let field_name = "my_field";
//     let ast = quote! {
//         int32 #field_name
//     };
//     file.write_all(ast.to_string().as_bytes()).unwrap();
//     println!("cargo:rerun-if-changed=build.rs");
// }

fn main() {}